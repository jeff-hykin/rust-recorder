//! One received message → the records a store writes: the per-format rules for codecs, TF splitting, images, source
//! times and the raw-LCM fallback. Runs on the encoding threads; the writer only appends.

use std::collections::BTreeMap;
use std::sync::Arc;

use anyhow::{anyhow, bail, ensure, Context, Result};

use crate::image::{self, ImageFormat, RawImage};
use crate::lcm::{self, Value};
use crate::ros2::{self, Mapping};
use crate::{Codec, Format, Options, StreamState};

const TF_TYPE: &str = "tf2_msgs.TFMessage";
const IMAGE_TYPE: &str = "sensor_msgs.Image";
const STRING_TYPE: &str = "std_msgs.String";
const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs.CompressedImage";

/// Where a record goes: an MCAP channel or a SQLite stream table. Built once per stream and variant.
#[derive(Debug)]
pub(crate) struct ChannelSpec {
    /// MCAP topic, or the SQLite table
    pub topic: String,
    pub message_encoding: String,
    /// (name, encoding, data)
    pub schema: Option<(String, String, Arc<[u8]>)>,
    pub metadata: BTreeMap<String, String>,
    pub sqlite: Option<SqliteStream>,
}

#[derive(Debug, Clone)]
pub(crate) struct SqliteStream {
    pub payload_type: String,
    pub codec_id: String,
    /// TF rows carry no reception tag or reception index (as dimos writes them)
    pub is_tf: bool,
}

pub(crate) struct Record {
    pub channel: Arc<ChannelSpec>,
    pub data: Vec<u8>,
    pub reception_ns: u64,
    pub source: SourceTime,
}

/// A record's source time: exact nanoseconds where the message carries a stamp, else seconds (JSON fields).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SourceTime {
    Stamp { sec: i64, nanosec: i64 },
    Seconds(f64),
}

impl SourceTime {
    pub fn seconds(self) -> f64 {
        match self {
            SourceTime::Stamp { sec, nanosec } => sec as f64 + nanosec as f64 / 1e9,
            SourceTime::Seconds(seconds) => seconds,
        }
    }

    /// MCAP time: finite, nonnegative, and within u64 nanoseconds; anything else is rejected, never clamped.
    pub fn mcap_nanos(self) -> Result<u64> {
        match self {
            SourceTime::Stamp { sec, nanosec } => {
                ensure!(sec >= 0 && (0..1_000_000_000).contains(&nanosec), "source time {sec}s {nanosec}ns is not a valid MCAP time");
                Ok(sec as u64 * 1_000_000_000 + nanosec as u64)
            }
            SourceTime::Seconds(seconds) => std::time::Duration::try_from_secs_f64(seconds)
                .ok()
                .and_then(|duration| u64::try_from(duration.as_nanos()).ok())
                .ok_or_else(|| anyhow!("source time {seconds} is not a valid MCAP time")),
        }
    }

    pub fn from_nanos(nanos: u64) -> SourceTime {
        SourceTime::Stamp { sec: (nanos / 1_000_000_000) as i64, nanosec: (nanos % 1_000_000_000) as i64 }
    }
}

/// What process() produced: records, plus a note when a message fell back to raw LCM.
pub(crate) struct Processed {
    pub records: Vec<Record>,
    pub fallback: Option<String>,
}

pub(crate) fn process(options: &Options, stream: &StreamState, payload: Vec<u8>, reception_ns: u64) -> Result<Processed> {
    if let Some(json) = &stream.json {
        let time = SourceTime::from_nanos(reception_ns);
        return json_record(options, stream, json, payload, reception_ns, time);
    }
    match options.format {
        Format::Sqlite | Format::DimosMcap => dimos_layout(options, stream, payload, reception_ns),
        Format::Ros2Mcap => ros2_layout(options, stream, payload, reception_ns),
    }
}

/// A pre-encoded JSON document (logs, labels): a json channel with its JSON Schema in MCAP, an LCM std_msgs.String
/// row in SQLite (readable by every dimos SqliteStore).
fn json_record(options: &Options, stream: &StreamState, json: &crate::JsonStream, payload: Vec<u8>, reception_ns: u64, time: SourceTime) -> Result<Processed> {
    let (variant, data) = if options.format == Format::Sqlite {
        let text = String::from_utf8(payload).context("JSON is not UTF-8")?;
        let message = Value::Struct(vec![Value::Str(text.into_bytes())]);
        ("lcm", lcm::registry().encode(STRING_TYPE, &message)?)
    } else {
        ("json", payload)
    };
    let channel = stream.channel(variant, || {
        let mut spec = dimos_channel(options, stream, variant, STRING_TYPE);
        if options.format != Format::Sqlite {
            spec.schema = Some((json.schema_name.clone(), "jsonschema".into(), json.schema.clone()));
        }
        Ok(spec)
    })?;
    Ok(Processed { records: vec![Record { channel, data, reception_ns, source: time }], fallback: None })
}

fn dimos_layout(options: &Options, stream: &StreamState, payload: Vec<u8>, reception_ns: u64) -> Result<Processed> {
    let msg_type = stream.msg_type.as_str();
    let reception = SourceTime::from_nanos(reception_ns);
    let codec = stream.resolve_codec(|| {
        if msg_type != IMAGE_TYPE {
            return Codec::Lcm;
        }
        // dimos stores images as JPEG by default; frames JPEG can't hold (float depth) go lossless instead
        let encoding = lcm::registry()
            .decode_prefix(IMAGE_TYPE, &payload, Some(image::image_fields().encoding))
            .ok()
            .and_then(|value| match value.fields()?.get(image::image_fields().encoding)? {
                Value::Str(bytes) => Some(String::from_utf8_lossy(bytes).into_owned()),
                _ => None,
            });
        match encoding {
            Some(encoding) if !image::dimos_jpeg_supports(&encoding) => Codec::Lz4Lcm,
            _ => Codec::Jpeg,
        }
    });
    if codec == Codec::Json {
        ensure!(msg_type == STRING_TYPE, "the json codec requires std_msgs.String");
        let message = lcm::registry().decode(STRING_TYPE, &payload).context("invalid String envelope")?;
        let Some(Value::Str(text)) = message.fields().and_then(|fields| fields.first()) else { bail!("invalid String envelope") };
        let document: serde_json::Value = serde_json::from_slice(text).context("invalid JSON document")?;
        let time = match &stream.spec.timestamp_field {
            Some(field) => {
                let seconds = document.get(field).and_then(serde_json::Value::as_f64).with_context(|| format!("JSON timestamp field {field:?} must be a number"))?;
                ensure!(seconds.is_finite(), "JSON timestamp must be finite");
                SourceTime::Seconds(seconds)
            }
            None => reception,
        };
        let channel = stream.channel("json", || {
            let mut spec = dimos_channel(options, stream, "json", msg_type);
            if let Some(schema) = &stream.spec.json_schema {
                spec.schema = Some((stream.stored_name.clone(), "jsonschema".into(), serde_json::to_vec(schema)?.into()));
            }
            Ok(spec)
        })?;
        return Ok(Processed { records: vec![Record { channel, data: text.clone(), reception_ns, source: time }], fallback: None });
    }
    let registry = lcm::registry();
    let known = registry.get(msg_type).is_some();
    let codec_id = codec.id();
    let channel = stream.channel(codec_id, || Ok(dimos_channel(options, stream, codec_id, msg_type)))?;
    let finish = |lcm_bytes: Vec<u8>| -> Result<Vec<u8>> {
        match codec {
            Codec::Lz4Lcm => lz4_frame(&lcm_bytes),
            _ => Ok(lcm_bytes),
        }
    };
    if msg_type == TF_TYPE && known {
        // each transform becomes its own observation at its own time (dimos' TF store layout)
        let message = registry.decode(TF_TYPE, &payload)?;
        let records = split_tf(&message)?
            .into_iter()
            .map(|(single, stamp)| {
                Ok(Record { channel: channel.clone(), data: finish(registry.encode(TF_TYPE, &single)?)?, reception_ns, source: positive_or(stamp, reception) })
            })
            .collect::<Result<Vec<_>>>()?;
        return Ok(Processed { records, fallback: None });
    }
    if codec == Codec::Jpeg {
        ensure!(msg_type == IMAGE_TYPE, "the jpeg codec requires sensor_msgs.Image");
        let mut image = registry.decode(IMAGE_TYPE, &payload).context("invalid LCM Image")?;
        let stamp = header_stamp(registry.get(IMAGE_TYPE).unwrap(), &image);
        image::dimos_jpeg(&mut image, options.dimos_jpeg_quality)?;
        let data = registry.encode(IMAGE_TYPE, &image)?;
        return Ok(Processed { records: vec![Record { channel, data, reception_ns, source: positive_or(stamp, reception) }], fallback: None });
    }
    let source = if known { positive_or(source_stamp(msg_type, &payload)?, reception) } else { reception };
    Ok(Processed { records: vec![Record { channel, data: finish(payload)?, reception_ns, source }], fallback: None })
}

fn ros2_layout(options: &Options, stream: &StreamState, payload: Vec<u8>, reception_ns: u64) -> Result<Processed> {
    match ros2_records(options, stream, &payload, reception_ns) {
        Ok(records) => Ok(Processed { records, fallback: None }),
        Err(error) if !options.strict_cdr => {
            // never dropped: kept as the LCM it arrived as, on its own channel of the same topic
            let channel = stream.channel("lcm", || Ok(raw_lcm_channel(stream)))?;
            let record = Record { channel, data: payload, reception_ns, source: SourceTime::from_nanos(reception_ns) };
            Ok(Processed { records: vec![record], fallback: Some(format!("{error:#}")) })
        }
        Err(error) => Err(error),
    }
}

fn ros2_records(options: &Options, stream: &StreamState, payload: &[u8], reception_ns: u64) -> Result<Vec<Record>> {
    let msg_type = stream.msg_type.as_str();
    let registry = lcm::registry();
    let definition = registry.get(msg_type).ok_or_else(|| anyhow!("{msg_type} is not a dimos-lcm type"))?;
    let map = match ros2::mapping(msg_type) {
        Mapping::Cdr(map) => map,
        Mapping::Unmapped(reason) => bail!("{msg_type} has no lossless ROS 2 mapping: {reason}"),
    };
    let value = registry.decode(msg_type, payload)?;
    let reception = SourceTime::from_nanos(reception_ns);
    let cdr_record = |map: &ros2::StructMap, value: &Value, stamp: Option<SourceTime>| -> Result<Record> {
        let source = stamp.unwrap_or(reception);
        source.mcap_nanos()?;
        let channel = stream.channel(&format!("cdr:{}", map.ros), || cdr_channel(stream, &map.ros))?;
        Ok(Record { channel, data: ros2::encode_cdr(map, value)?, reception_ns, source })
    };
    if msg_type == TF_TYPE {
        return split_tf(&value)?.into_iter().map(|(single, stamp)| cdr_record(&map, &single, stamp)).collect();
    }
    let stamp = header_stamp(definition, &value).or_else(|| timestamp_member(definition, &value));
    if msg_type == IMAGE_TYPE {
        let image = RawImage::from_value(&value)?;
        let compressed = match image.container_format() {
            Some(format) => Some((format, image.data.to_vec())),
            None if options.image_format != ImageFormat::Raw => image::compress(&image, options.image_format, options.jpeg_quality).map(|data| (options.image_format.label(), data)),
            None => None,
        };
        if let Some((format, data)) = compressed {
            let header = value.fields().unwrap()[image::image_fields().header].clone();
            let message = compressed_image(header, format, data)?;
            let Mapping::Cdr(compressed_map) = ros2::mapping(COMPRESSED_IMAGE_TYPE) else { bail!("CompressedImage has no ROS 2 mapping") };
            return Ok(vec![cdr_record(&compressed_map, &message, stamp)?]);
        }
    }
    Ok(vec![cdr_record(&map, &value, stamp)?])
}

/// An LCM-shaped sensor_msgs.CompressedImage, so it goes through the same mapping as one received on the wire.
fn compressed_image(header: Value, format: &str, data: Vec<u8>) -> Result<Value> {
    let definition = lcm::registry().get(COMPRESSED_IMAGE_TYPE).ok_or_else(|| anyhow!("CompressedImage is not vendored"))?;
    let mut fields = vec![Value::Int(0); definition.members.len()];
    let index = |name: &str| definition.member_index(name).ok_or_else(|| anyhow!("CompressedImage.{name}"));
    fields[index("header")?] = header;
    fields[index("format")?] = Value::Str(format.as_bytes().to_vec());
    fields[index("data_length")?] = Value::Int(data.len() as i64);
    fields[index("data")?] = Value::Bytes(data);
    Ok(Value::Struct(fields))
}

/// TFMessage → one single-transform TFMessage per transform, each with its own stamp.
fn split_tf(message: &Value) -> Result<Vec<(Value, Option<SourceTime>)>> {
    let tf = lcm::registry().get(TF_TYPE).unwrap();
    let transforms_index = tf.member_index("transforms").ok_or_else(|| anyhow!("TFMessage.transforms"))?;
    let length_index = tf.member_index("transforms_length");
    let Some(Value::Array(transforms)) = message.fields().and_then(|fields| fields.get(transforms_index)) else { bail!("TFMessage.transforms is not an array") };
    let stamped = lcm::registry().get("geometry_msgs.TransformStamped").ok_or_else(|| anyhow!("TransformStamped is not vendored"))?;
    Ok(transforms
        .iter()
        .map(|transform| {
            let mut fields = vec![Value::Int(0); tf.members.len()];
            if let Some(length_index) = length_index {
                fields[length_index] = Value::Int(1);
            }
            fields[transforms_index] = Value::Array(vec![transform.clone()]);
            (Value::Struct(fields), header_stamp(stamped, transform))
        })
        .collect())
}

/// The `header.stamp` of a decoded message, if its type has a std_msgs Header.
fn header_stamp(definition: &lcm::Struct, value: &Value) -> Option<SourceTime> {
    let index = definition.members.iter().position(|member| member.name == "header" && member.dims.is_empty() && member.ty == lcm::MemberType::Struct("std_msgs.Header".into()))?;
    let header = lcm::registry().get("std_msgs.Header")?;
    let stamp = value.fields()?.get(index)?.fields()?.get(header.member_index("stamp")?)?;
    time_of(stamp)
}

/// foxglove-style `timestamp` member (e.g. CompressedVideo).
fn timestamp_member(definition: &lcm::Struct, value: &Value) -> Option<SourceTime> {
    let index = definition.members.iter().position(|member| {
        member.name == "timestamp" && member.dims.is_empty() && matches!(&member.ty, lcm::MemberType::Struct(name) if name.ends_with(".Time"))
    })?;
    time_of(value.fields()?.get(index)?)
}

fn time_of(time: &Value) -> Option<SourceTime> {
    let fields = time.fields()?;
    Some(SourceTime::Stamp { sec: fields.first()?.as_i64()?, nanosec: fields.get(1)?.as_i64()? })
}

/// The source time dimos' recorder uses: the header stamp when its seconds are positive, else reception.
fn positive_or(stamp: Option<SourceTime>, reception: SourceTime) -> SourceTime {
    match stamp {
        Some(SourceTime::Stamp { sec, .. }) if sec > 0 => stamp.unwrap(),
        _ => reception,
    }
}

/// The stamp of a raw LCM message, decoding only up to its header (or `timestamp`).
fn source_stamp(msg_type: &str, payload: &[u8]) -> Result<Option<SourceTime>> {
    let registry = lcm::registry();
    let definition = registry.get(msg_type).unwrap();
    let stop = definition.members.iter().position(|member| (member.name == "header" || member.name == "timestamp") && member.dims.is_empty());
    let Some(stop) = stop else {
        // still check it is what it claims to be
        registry.decode(msg_type, payload).with_context(|| format!("invalid LCM {msg_type}"))?;
        return Ok(None);
    };
    let value = registry.decode_prefix(msg_type, payload, Some(stop)).with_context(|| format!("invalid LCM {msg_type}"))?;
    Ok(header_stamp(definition, &value).or_else(|| timestamp_member(definition, &value)))
}

pub(crate) fn lz4_frame(data: &[u8]) -> Result<Vec<u8>> {
    use std::io::Write;
    let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
    encoder.write_all(data).context("LZ4 compression failed")?;
    encoder.finish().context("LZ4 frame finalization failed")
}

/// The dimos-layout channel (cc's store/mcap.rs metadata) and SQLite registration for a stream.
pub(crate) fn dimos_channel(options: &Options, stream: &StreamState, codec_id: &str, msg_type: &str) -> ChannelSpec {
    let payload_type = crate::payload_type_for(msg_type);
    ChannelSpec {
        topic: stream.stored_name.clone(),
        message_encoding: codec_id.to_string(),
        schema: None,
        metadata: dimos_metadata(stream, &payload_type),
        sqlite: (options.format == Format::Sqlite).then(|| SqliteStream { payload_type, codec_id: codec_id.to_string(), is_tf: msg_type == TF_TYPE }),
    }
}

fn dimos_metadata(stream: &StreamState, payload_type: &str) -> BTreeMap<String, String> {
    BTreeMap::from([
        ("dimos.payload_type".to_string(), payload_type.to_string()),
        ("dimos.stream_name".to_string(), stream.stored_name.clone()),
        ("dimos.port".to_string(), stream.spec.port.clone().unwrap_or_else(|| stream.stored_name.clone())),
        ("dimos.observation_time".to_string(), "publish_time".to_string()),
    ])
}

pub(crate) fn cdr_channel(stream: &StreamState, ros_name: &str) -> Result<ChannelSpec> {
    let text = ros2::registry().definition(ros_name)?;
    Ok(ChannelSpec {
        topic: stream.stored_name.clone(),
        message_encoding: "cdr".into(),
        schema: Some((ros_name.to_string(), "ros2msg".into(), Arc::from(text.as_bytes()))),
        metadata: dimos_metadata(stream, &crate::payload_type_for(&stream.msg_type)),
        sqlite: None,
    })
}

pub(crate) fn raw_lcm_channel(stream: &StreamState) -> ChannelSpec {
    let mut metadata = dimos_metadata(stream, &crate::payload_type_for(&stream.msg_type));
    // the LCM type is the only clue a reader has about what the bytes are
    metadata.insert("lcm_type".into(), stream.msg_type.clone());
    ChannelSpec { topic: stream.stored_name.clone(), message_encoding: "lcm".into(), schema: None, metadata, sqlite: None }
}

/// The channel a stream with no messages is declared with, so every added stream appears in the recording.
pub(crate) fn empty_channel(options: &Options, stream: &StreamState) -> Result<(String, Arc<ChannelSpec>)> {
    if let Some(json) = &stream.json {
        let variant = if options.format == Format::Sqlite { "lcm" } else { "json" };
        let channel = stream.channel(variant, || {
            let mut spec = dimos_channel(options, stream, variant, STRING_TYPE);
            if options.format != Format::Sqlite {
                spec.schema = Some((json.schema_name.clone(), "jsonschema".into(), json.schema.clone()));
            }
            Ok(spec)
        })?;
        return Ok((variant.into(), channel));
    }
    match options.format {
        Format::Sqlite | Format::DimosMcap => {
            let codec = stream.resolve_codec(|| if stream.msg_type == IMAGE_TYPE { Codec::Jpeg } else { Codec::Lcm });
            let variant = codec.id();
            Ok((variant.into(), stream.channel(variant, || Ok(dimos_channel(options, stream, variant, &stream.msg_type)))?))
        }
        Format::Ros2Mcap => match ros2::mapping(&stream.msg_type) {
            Mapping::Cdr(map) => {
                let variant = format!("cdr:{}", map.ros);
                Ok((variant.clone(), stream.channel(&variant, || cdr_channel(stream, &map.ros))?))
            }
            Mapping::Unmapped(_) => Ok(("lcm".into(), stream.channel("lcm", || Ok(raw_lcm_channel(stream)))?)),
        },
    }
}
