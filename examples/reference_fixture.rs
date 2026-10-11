//! Writes recordings plus what readers should find in them, for tests/reference (rosbags, Foxglove's decoders, dimos).
//! `cargo run --example reference_fixture -- <dir>`

use std::collections::BTreeMap;

use rust_recorder::lcm::{self, Value};
use rust_recorder::ros2::{mapping, to_ros_json, Mapping};
use rust_recorder::testing::sample_value;
use rust_recorder::{Codec, Compression, Format, Options, Recorder, StreamSpec};
use serde_json::json;

fn main() -> anyhow::Result<()> {
    let directory = std::path::PathBuf::from(std::env::args().nth(1).unwrap_or_else(|| "fixtures".into()));
    std::fs::create_dir_all(&directory)?;
    let registry = lcm::registry();

    // ROS 2: one sample of every type with a CDR mapping, three times (different seeds)
    let mut options = Options::new(Format::Ros2Mcap, directory.join("ros2.mcap"));
    options.compression = Compression::None;
    let recorder = Recorder::open(options)?;
    let mut expected: BTreeMap<String, serde_json::Value> = BTreeMap::new();
    let mut names: Vec<&String> = registry.structs.keys().collect();
    names.sort();
    for name in names {
        let Mapping::Cdr(map) = mapping(name) else { continue };
        let topic = format!("/{}", name.replace('.', "/"));
        let stream = recorder.add_stream(StreamSpec::new(&topic, name.as_str()))?;
        let mut messages = Vec::new();
        for seed in 0..3u64 {
            let value = sample_value(name, seed * 7919 + name.len() as u64);
            recorder.record(&stream, &registry.encode(name, &value)?);
            if name == "tf2_msgs.TFMessage" {
                let Value::Struct(fields) = &value else { unreachable!() };
                let Value::Array(transforms) = &fields[1] else { unreachable!() };
                for transform in transforms {
                    let single = Value::Struct(vec![Value::Int(1), Value::Array(vec![transform.clone()])]);
                    messages.push(to_ros_json(&map, Some(&single))?);
                }
            } else {
                messages.push(to_ros_json(&map, Some(&value))?);
            }
        }
        expected.insert(topic, json!({ "schema": map.ros, "messages": messages }));
    }
    let stats = recorder.finish()?;
    anyhow::ensure!(stats.rejected == 0 && stats.fallback == 0 && stats.dropped == 0, "{stats:#?}");
    std::fs::write(directory.join("ros2.expected.json"), serde_json::to_vec_pretty(&expected)?)?;

    // dimos layouts: the streams a dimos robot records, through dimos' own readers
    for (format, file) in [(Format::Sqlite, "dimos.db"), (Format::DimosMcap, "dimos.mcap")] {
        let recorder = Recorder::open(Options::new(format, directory.join(file)))?;
        let mut expected: BTreeMap<String, serde_json::Value> = BTreeMap::new();
        let streams = [
            ("odom", "nav_msgs.Odometry", None),
            ("pose", "geometry_msgs.PoseStamped", None),
            ("imu", "sensor_msgs.Imu", Some(Codec::Lz4Lcm)),
            ("lidar", "sensor_msgs.PointCloud2", Some(Codec::Lz4Lcm)),
            ("joints", "sensor_msgs.JointState", None),
            ("cmd_vel", "geometry_msgs.Twist", None),
            ("tf", "tf2_msgs.TFMessage", None),
        ];
        for (stream_name, msg_type, codec) in streams {
            let mut spec = StreamSpec::new(stream_name, msg_type);
            spec.codec = codec;
            let stream = recorder.add_stream(spec)?;
            let mut times = Vec::new();
            for seed in 0..4u64 {
                let mut value = sample_value(msg_type, seed + 11);
                if msg_type == "sensor_msgs.PointCloud2" {
                    make_xyz_cloud(&mut value);
                }
                recorder.record_at(&stream, registry.encode(msg_type, &value)?, 1_800_000_000_000_000_000 + seed);
                times.extend(stamps(msg_type, &value));
            }
            expected.insert(stream_name.into(), json!({ "payload_type": rust_recorder::payload_type_for(msg_type), "ts": times }));
        }
        // images: an rgb8 frame (jpeg codec by default) and a 32-bit depth frame (lossless lz4+lcm by default)
        for (stream_name, encoding, bytes_per_pixel) in [("color_image", "rgb8", 3usize), ("depth_image", "32FC1", 4)] {
            let stream = recorder.add_stream(StreamSpec::new(stream_name, "sensor_msgs.Image"))?;
            let mut value = sample_value("sensor_msgs.Image", 5);
            let Value::Struct(fields) = &mut value else { unreachable!() };
            let (width, height) = (16usize, 8usize);
            fields[2] = Value::Int(height as i64);
            fields[3] = Value::Int(width as i64);
            fields[4] = Value::Str(encoding.as_bytes().to_vec());
            fields[5] = Value::Int(0);
            fields[6] = Value::Int((width * bytes_per_pixel) as i64);
            fields[7] = Value::Bytes((0..width * height * bytes_per_pixel).map(|index| (index * 37 % 251) as u8).collect());
            recorder.record_at(&stream, registry.encode("sensor_msgs.Image", &value)?, 1_800_000_000_000_000_000);
            expected.insert(stream_name.into(), json!({ "payload_type": "dimos.msgs.sensor_msgs.Image.Image", "ts": stamps("sensor_msgs.Image", &value), "width": width, "height": height }));
        }
        // a late stream, added after messages were already written
        let late = recorder.add_stream(StreamSpec::new("late_odom", "nav_msgs.Odometry"))?;
        let value = sample_value("nav_msgs.Odometry", 99);
        recorder.record(&late, &registry.encode("nav_msgs.Odometry", &value)?);
        expected.insert("late_odom".into(), json!({ "payload_type": "dimos.msgs.nav_msgs.Odometry.Odometry", "ts": stamps("nav_msgs.Odometry", &value) }));
        let stats = recorder.finish()?;
        anyhow::ensure!(stats.rejected == 0 && stats.dropped == 0, "{stats:#?}");
        std::fs::write(directory.join(format!("{file}.expected.json")), serde_json::to_vec_pretty(&expected)?)?;
    }
    println!("fixtures in {}", directory.display());
    Ok(())
}

/// The source times dimos' readers should report: the header stamp (one per transform for TF).
fn stamps(msg_type: &str, value: &Value) -> Vec<f64> {
    let time = |stamp: &Value| {
        let fields = stamp.fields().unwrap();
        fields[0].as_i64().unwrap() as f64 + fields[1].as_i64().unwrap() as f64 / 1e9
    };
    let header_time = |header: &Value| time(&header.fields().unwrap()[1]);
    let fields = value.fields().unwrap();
    match msg_type {
        "tf2_msgs.TFMessage" => {
            let Value::Array(transforms) = &fields[1] else { unreachable!() };
            transforms.iter().map(|transform| header_time(&transform.fields().unwrap()[0])).collect()
        }
        "geometry_msgs.Twist" => vec![1_800_000_000.0],
        _ => {
            let definition = lcm::registry().get(msg_type).unwrap();
            vec![header_time(&fields[definition.member_index("header").unwrap()])]
        }
    }
}

/// A real two-point x/y/z float32 cloud (dimos' PointCloud2 requires x, y and z fields).
fn make_xyz_cloud(value: &mut Value) {
    let registry = lcm::registry();
    let cloud = registry.get("sensor_msgs.PointCloud2").unwrap();
    let field = registry.get("sensor_msgs.PointField").unwrap();
    let index = |name: &str| cloud.member_index(name).unwrap();
    let Value::Struct(fields) = value else { unreachable!() };
    let point_fields = ["x", "y", "z"]
        .iter()
        .enumerate()
        .map(|(position, name)| {
            let mut point_field = vec![Value::Int(0); field.members.len()];
            point_field[field.member_index("name").unwrap()] = Value::Str(name.as_bytes().to_vec());
            point_field[field.member_index("offset").unwrap()] = Value::Int(position as i64 * 4);
            point_field[field.member_index("datatype").unwrap()] = Value::Int(7);
            point_field[field.member_index("count").unwrap()] = Value::Int(1);
            Value::Struct(point_field)
        })
        .collect();
    fields[index("fields")] = Value::Array(point_fields);
    fields[index("height")] = Value::Int(1);
    fields[index("width")] = Value::Int(2);
    fields[index("is_bigendian")] = Value::Bool(false);
    fields[index("point_step")] = Value::Int(12);
    fields[index("row_step")] = Value::Int(24);
    fields[index("data")] = Value::Bytes([1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0].iter().flat_map(|number| number.to_le_bytes()).collect());
    fields[index("is_dense")] = Value::Bool(true);
}
