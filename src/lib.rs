// Copyright 2026 Dimensional Inc. (the dimos native recorder this derives from)
// Copyright 2026 Jeff Hykin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Records dimos streams (LCM bytes plus a type name) to a dimos SqliteStore `.db`, a dimos-layout MCAP, or a ROS 2
//! MCAP (CDR with full `ros2msg` schemas) that opens in Foxglove. No transport dependency: callers hand it bytes;
//! [`zenoh_recorder!`] wires it to whichever zenoh version the caller uses.
//!
//! ```no_run
//! use rust_recorder::{Format, Options, Recorder, StreamSpec};
//! let recorder = Recorder::open(Options::new(Format::Ros2Mcap, "run.mcap")).unwrap();
//! let odom = recorder.add_stream(StreamSpec::new("/odom", "nav_msgs.Odometry")).unwrap();
//! # let lcm_bytes: Vec<u8> = vec![];
//! recorder.record(&odom, &lcm_bytes);
//! let stats = recorder.finish().unwrap();
//! println!("{} written, {} dropped", stats.written, stats.dropped);
//! ```

mod definitions {
    include!(concat!(env!("OUT_DIR"), "/definitions.rs"));
}
pub mod image;
pub mod lcm;
mod process;
pub mod ros2;
#[doc(hidden)]
pub mod testing;
mod store;
mod zenoh;

use std::collections::{BTreeMap, HashMap};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub use ::anyhow;
use ::anyhow::{anyhow, bail, Context, Result};
use crossbeam_channel::{unbounded, Receiver, RecvTimeoutError, Sender};
use serde::{Deserialize, Serialize};

pub use image::ImageFormat;
use process::ChannelSpec;
pub use store::{companions, recover, Recovered};

/// The file a recording is written as.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Format {
    /// a dimos SqliteStore `.db` (`SqliteStore(path=…)`, `dimos --replay`)
    Sqlite,
    /// MCAP in dimos' storage layout (profile "dimos": lcm / jpeg / lz4+lcm / json channels), what dimos' McapStore reads
    DimosMcap,
    /// MCAP with profile "ros2": CDR with full ros2msg schemas (Foxglove, rosbag2 tooling); raw LCM where no ROS 2 type fits
    #[default]
    Ros2Mcap,
}

impl Format {
    pub fn extension(self) -> &'static str {
        match self {
            Format::Sqlite => "db",
            Format::DimosMcap | Format::Ros2Mcap => "mcap",
        }
    }
}

/// MCAP chunk compression.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Compression {
    None,
    Lz4,
    #[default]
    Zstd,
}

/// A dimos storage codec (dimos stores only; ROS 2 recordings are CDR).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Codec {
    Lcm,
    Jpeg,
    #[serde(rename = "lz4+lcm")]
    Lz4Lcm,
    Json,
}

impl Codec {
    pub fn id(self) -> &'static str {
        match self {
            Codec::Lcm => "lcm",
            Codec::Jpeg => "jpeg",
            Codec::Lz4Lcm => "lz4+lcm",
            Codec::Json => "json",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Options {
    pub format: Format,
    pub path: PathBuf,
    /// MCAP chunk compression (chunks are closed every second either way)
    pub compression: Compression,
    /// ROS 2 recordings: how raw image frames are stored
    pub image_format: ImageFormat,
    /// ROS 2 recordings: JPEG quality for `image_format: Jpeg`
    pub jpeg_quality: u8,
    /// dimos stores: JPEG quality of the `jpeg` codec (dimos' JpegCodec default)
    pub dimos_jpeg_quality: u8,
    /// ROS 2 recordings: refuse (count as rejected) what can't be CDR instead of keeping it as a raw LCM channel
    pub strict_cdr: bool,
    /// threads decoding and re-encoding messages (one writer keeps arrival order)
    pub encoding_threads: usize,
    /// messages waiting to be written beyond which new ones are dropped (and counted)
    pub queue_messages: usize,
    /// bytes waiting to be written beyond which new messages are dropped (and counted)
    pub queue_bytes: usize,
}

impl Options {
    pub fn new(format: Format, path: impl Into<PathBuf>) -> Options {
        Options {
            format,
            path: path.into(),
            compression: Compression::Zstd,
            image_format: ImageFormat::Raw,
            jpeg_quality: 90,
            dimos_jpeg_quality: 50,
            strict_cdr: false,
            encoding_threads: std::thread::available_parallelism().map(|count| count.get().clamp(1, 4)).unwrap_or(2),
            queue_messages: 1024,
            queue_bytes: 512 << 20,
        }
    }
}

/// A stream to record: a name (an MCAP topic; dimos stores make it an identifier) and its dimos-lcm type.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct StreamSpec {
    pub name: String,
    /// `sensor_msgs.Image`, `sensor_msgs/msg/Image` or `dimos.msgs.sensor_msgs.Image.Image`
    pub msg_type: String,
    /// dimos stores: the storage codec (default: jpeg for images, lossless lz4+lcm for frames JPEG can't hold, lcm otherwise)
    #[serde(default)]
    pub codec: Option<Codec>,
    /// the dimos port it came from (channel metadata; defaults to the name)
    #[serde(default)]
    pub port: Option<String>,
    /// json codec: the document field holding the source time (seconds)
    #[serde(default)]
    pub timestamp_field: Option<String>,
    /// json codec: a JSON Schema embedded in MCAP
    #[serde(default)]
    pub json_schema: Option<serde_json::Value>,
}

impl StreamSpec {
    pub fn new(name: impl Into<String>, msg_type: impl Into<String>) -> StreamSpec {
        StreamSpec { name: name.into(), msg_type: msg_type.into(), ..StreamSpec::default() }
    }

    /// The stream a dimos zenoh key carries (`dimos/<port>/<pkg.Type>`), named `/<port>`.
    pub fn from_dimos_key(key: &str) -> Option<StreamSpec> {
        let (port, msg_type) = split_dimos_key(key)?;
        Some(StreamSpec { name: format!("/{port}"), msg_type, port: Some(port), ..StreamSpec::default() })
    }

    pub fn with_codec(mut self, codec: Codec) -> StreamSpec {
        self.codec = Some(codec);
        self
    }
}

/// `dimos/<port>/<pkg.Type>` → (port, `pkg.Type`); None for anything that isn't a dimos data key.
pub fn split_dimos_key(key: &str) -> Option<(String, String)> {
    let rest = key.strip_prefix("dimos/")?;
    let (port, msg_type) = rest.rsplit_once('/')?;
    let (package, name) = msg_type.split_once('.')?;
    if port.is_empty() || package.is_empty() || name.is_empty() || name.contains('.') || key.contains('*') {
        return None;
    }
    Some((port.to_string(), msg_type.to_string()))
}

/// Normalizes any spelling of a type to dimos-lcm's `package.Name`.
pub fn lcm_type_name(msg_type: &str) -> String {
    if let Some(rest) = msg_type.strip_prefix("dimos.msgs.") {
        let parts: Vec<&str> = rest.split('.').collect();
        if parts.len() >= 2 {
            return format!("{}.{}", parts[0], parts[1]);
        }
    }
    msg_type.replace("/msg/", "/").replace('/', ".")
}

/// The payload class dimos' stores record for a type: `dimos.msgs.<package>.<Name>.<Name>`.
pub fn payload_type_for(msg_type: &str) -> String {
    let lcm_name = lcm_type_name(msg_type);
    let (package, name) = lcm_name.split_once('.').unwrap_or(("", &lcm_name));
    format!("dimos.msgs.{package}.{name}.{name}")
}

/// A name dimos' stores accept (a SQL identifier): `/camera/color` → `camera_color`.
pub fn stream_name_for(name: &str) -> String {
    let mut out: String = name.trim_matches('/').chars().map(|character| if character.is_ascii_alphanumeric() { character } else { '_' }).collect();
    if out.is_empty() || out.starts_with(|character: char| character.is_ascii_digit()) {
        out.insert(0, '_');
    }
    out
}

pub(crate) fn validate_identifier(name: &str) -> Result<()> {
    let mut characters = name.chars();
    let valid = characters.next().is_some_and(|first| first == '_' || first.is_ascii_alphabetic()) && characters.all(|character| character == '_' || character.is_ascii_alphanumeric());
    if !valid {
        bail!("invalid stream name {name:?}");
    }
    Ok(())
}

pub(crate) struct JsonStream {
    pub schema_name: String,
    pub schema: Arc<[u8]>,
}

#[derive(Default)]
struct Counters {
    received: AtomicU64,
    written: AtomicU64,
    bytes: AtomicU64,
    dropped: AtomicU64,
    skipped: AtomicU64,
    rejected: AtomicU64,
    fallback: AtomicU64,
}

pub(crate) struct StreamState {
    pub spec: StreamSpec,
    /// dimos-lcm `package.Name`
    pub msg_type: String,
    /// MCAP topic / SQLite table
    pub stored_name: String,
    pub json: Option<JsonStream>,
    codec: OnceLock<Codec>,
    channels: Mutex<HashMap<String, Arc<ChannelSpec>>>,
    counters: Counters,
    last_error: Mutex<Option<String>>,
}

impl StreamState {
    pub(crate) fn channel(&self, variant: &str, build: impl FnOnce() -> Result<ChannelSpec>) -> Result<Arc<ChannelSpec>> {
        let mut channels = self.channels.lock().unwrap();
        if let Some(channel) = channels.get(variant) {
            return Ok(channel.clone());
        }
        let channel = Arc::new(build()?);
        channels.insert(variant.to_string(), channel.clone());
        Ok(channel)
    }

    /// The codec the stream was given, or the one chosen for it the first time it was needed.
    pub(crate) fn resolve_codec(&self, choose: impl FnOnce() -> Codec) -> Codec {
        if let Some(codec) = self.spec.codec {
            return codec;
        }
        *self.codec.get_or_init(choose)
    }

    fn note_error(&self, error: String) {
        *self.last_error.lock().unwrap() = Some(error);
    }
}

/// A stream added to a recorder; cheap to clone.
#[derive(Clone)]
pub struct Stream(Arc<StreamState>);

impl Stream {
    /// The topic (MCAP) or table (SQLite) it is recorded as.
    pub fn stored_name(&self) -> &str {
        &self.0.stored_name
    }

    pub fn msg_type(&self) -> &str {
        &self.0.msg_type
    }
}

#[derive(Clone, Debug, Default, Serialize, PartialEq)]
pub struct StreamStats {
    pub name: String,
    pub stored_name: String,
    pub msg_type: String,
    /// messages handed to `record`
    pub received: u64,
    /// records written (a TF message becomes one per transform)
    pub written: u64,
    pub bytes: u64,
    /// refused because the write queue was full (or the recorder had stopped)
    pub dropped: u64,
    /// left out by the caller (`skip`, e.g. a max rate)
    pub skipped: u64,
    /// invalid payloads or times that could not be written in this format
    pub rejected: u64,
    /// ROS 2 recordings: messages kept as raw LCM because they could not be CDR
    pub fallback: u64,
    pub last_error: Option<String>,
}

#[derive(Clone, Debug, Default, Serialize, PartialEq)]
pub struct Stats {
    pub path: String,
    pub received: u64,
    pub written: u64,
    pub bytes: u64,
    pub dropped: u64,
    pub skipped: u64,
    pub rejected: u64,
    pub fallback: u64,
    pub seconds: f64,
    /// a write failure (disk full, …) that stopped the recording
    pub failure: Option<String>,
    pub streams: Vec<StreamStats>,
}

struct Job {
    sequence: u64,
    stream: Arc<StreamState>,
    payload: Vec<u8>,
    reception_ns: u64,
    /// bytes held against the queue budget (0 for JSON writes, which are never dropped)
    budget: usize,
}

struct Done {
    sequence: u64,
    stream: Arc<StreamState>,
    result: Result<process::Processed>,
    budget: usize,
}

struct Shared {
    options: Options,
    streams: Mutex<Vec<Arc<StreamState>>>,
    pending_messages: AtomicUsize,
    pending_bytes: AtomicUsize,
    stopped: AtomicBool,
    failure: Mutex<Option<String>>,
}

struct Submission {
    sender: Sender<Job>,
    next_sequence: u64,
}

/// A recording in progress. Share it (`Arc<Recorder>`) between the threads that receive messages.
pub struct Recorder {
    shared: Arc<Shared>,
    submission: Mutex<Option<Submission>>,
    workers: Mutex<Vec<JoinHandle<()>>>,
    writer: Mutex<Option<JoinHandle<()>>>,
    started: Instant,
    final_stats: Mutex<Option<Stats>>,
}

const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

impl Recorder {
    /// Creates the file (replacing one at the path) and starts the encoding threads and the writer.
    pub fn open(options: Options) -> Result<Recorder> {
        if !image::format_available(options.image_format) {
            bail!("image format {:?} is not compiled in (enable the rust-recorder feature)", options.image_format);
        }
        if options.format == Format::Sqlite {
            for path in std::iter::once(options.path.clone()).chain(companions(&options.path)) {
                if path.exists() {
                    std::fs::remove_file(&path).with_context(|| format!("could not replace {}", path.display()))?;
                }
            }
        }
        let store = store::open(&options)?;
        let shared = Arc::new(Shared {
            options: options.clone(),
            streams: Mutex::default(),
            pending_messages: AtomicUsize::new(0),
            pending_bytes: AtomicUsize::new(0),
            stopped: AtomicBool::new(false),
            failure: Mutex::default(),
        });
        let (job_sender, job_receiver) = unbounded::<Job>();
        let (done_sender, done_receiver) = unbounded::<Done>();
        let mut workers = Vec::new();
        for index in 0..options.encoding_threads.max(1) {
            let (jobs, done, shared) = (job_receiver.clone(), done_sender.clone(), shared.clone());
            workers.push(std::thread::Builder::new().name(format!("recorder-encode-{index}")).spawn(move || encode_loop(&shared, jobs, done))?);
        }
        drop(done_sender);
        let writer_shared = shared.clone();
        let writer = std::thread::Builder::new().name("recorder-writer".into()).spawn(move || {
            if let Err(error) = writer_loop(&writer_shared, store, done_receiver) {
                writer_shared.stopped.store(true, Ordering::SeqCst);
                writer_shared.failure.lock().unwrap().get_or_insert(format!("{error:#}"));
            }
        })?;
        Ok(Recorder {
            shared,
            submission: Mutex::new(Some(Submission { sender: job_sender, next_sequence: 0 })),
            workers: Mutex::new(workers),
            writer: Mutex::new(Some(writer)),
            started: Instant::now(),
            final_stats: Mutex::default(),
        })
    }

    pub fn options(&self) -> &Options {
        &self.shared.options
    }

    /// Adds a stream, at any time while recording. Adding the same name and type again returns the existing stream.
    pub fn add_stream(&self, spec: StreamSpec) -> Result<Stream> {
        self.add(spec, None)
    }

    /// A stream of pre-encoded JSON documents (logs, labels) with their JSON Schema; written with `record_json`.
    pub fn add_json_stream(&self, name: &str, schema_name: &str, json_schema: &[u8]) -> Result<Stream> {
        let spec = StreamSpec::new(name, "std_msgs.String");
        self.add(spec, Some(JsonStream { schema_name: schema_name.to_string(), schema: Arc::from(json_schema) }))
    }

    fn add(&self, spec: StreamSpec, json: Option<JsonStream>) -> Result<Stream> {
        let msg_type = lcm_type_name(&spec.msg_type);
        if spec.codec == Some(Codec::Json) && msg_type != "std_msgs.String" {
            bail!("the json codec requires std_msgs.String, not {msg_type}");
        }
        if spec.codec != Some(Codec::Json) && (spec.timestamp_field.is_some() || spec.json_schema.is_some()) {
            bail!("timestamp_field and json_schema need the json codec");
        }
        if spec.codec == Some(Codec::Jpeg) && msg_type != "sensor_msgs.Image" {
            bail!("the jpeg codec requires sensor_msgs.Image, not {msg_type}");
        }
        let mut streams = self.shared.streams.lock().unwrap();
        let dimos_store = self.shared.options.format != Format::Ros2Mcap;
        let mut stored_name = if dimos_store { stream_name_for(&spec.name) } else { spec.name.clone() };
        if let Some(existing) = streams.iter().find(|stream| stream.spec.name == spec.name && stream.msg_type == msg_type) {
            return Ok(Stream(existing.clone()));
        }
        if dimos_store {
            // one table per stream: a second type on the same name gets its own (`name_Type`, as dimos' McapStore names it)
            if streams.iter().any(|stream| stream.stored_name == stored_name) {
                stored_name = format!("{stored_name}_{}", msg_type.rsplit('.').next().unwrap_or("other"));
            }
            let base = stored_name.clone();
            let mut suffix = 2;
            while streams.iter().any(|stream| stream.stored_name == stored_name) {
                stored_name = format!("{base}_{suffix}");
                suffix += 1;
            }
            validate_identifier(&stored_name)?;
        }
        let state = Arc::new(StreamState {
            spec,
            msg_type,
            stored_name,
            json,
            codec: OnceLock::new(),
            channels: Mutex::default(),
            counters: Counters::default(),
            last_error: Mutex::default(),
        });
        streams.push(state.clone());
        Ok(Stream(state))
    }

    /// Records one message (its LCM bytes) received now. Never blocks: when the queue is full the message is dropped
    /// and counted; returns whether it was taken.
    pub fn record(&self, stream: &Stream, payload: &[u8]) -> bool {
        self.record_at(stream, payload.to_vec(), now_nanos())
    }

    /// Like `record`, with the bytes moved in and the reception time (ns since the Unix epoch) given.
    pub fn record_at(&self, stream: &Stream, payload: Vec<u8>, reception_ns: u64) -> bool {
        let counters = &stream.0.counters;
        counters.received.fetch_add(1, Ordering::Relaxed);
        let shared = &self.shared;
        let size = payload.len();
        let over_messages = shared.pending_messages.fetch_add(1, Ordering::SeqCst) >= shared.options.queue_messages;
        let over_bytes = shared.pending_bytes.fetch_add(size, Ordering::SeqCst).saturating_add(size) > shared.options.queue_bytes;
        if over_messages || over_bytes || shared.stopped.load(Ordering::SeqCst) {
            shared.pending_messages.fetch_sub(1, Ordering::SeqCst);
            shared.pending_bytes.fetch_sub(size, Ordering::SeqCst);
            counters.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        if !self.submit(stream, payload, reception_ns, size) {
            shared.pending_messages.fetch_sub(1, Ordering::SeqCst);
            shared.pending_bytes.fetch_sub(size, Ordering::SeqCst);
            counters.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        true
    }

    /// Writes a JSON document on a stream from `add_json_stream`, at `time_ns`. Never dropped for a full queue.
    pub fn record_json(&self, stream: &Stream, json: Vec<u8>, time_ns: u64) -> Result<()> {
        if stream.0.json.is_none() {
            bail!("{} is not a JSON stream (use add_json_stream)", stream.0.spec.name);
        }
        stream.0.counters.received.fetch_add(1, Ordering::Relaxed);
        self.shared.pending_messages.fetch_add(1, Ordering::SeqCst);
        if !self.submit(stream, json, time_ns, 0) {
            self.shared.pending_messages.fetch_sub(1, Ordering::SeqCst);
            stream.0.counters.dropped.fetch_add(1, Ordering::Relaxed);
            bail!("the recording has stopped");
        }
        Ok(())
    }

    fn submit(&self, stream: &Stream, payload: Vec<u8>, reception_ns: u64, budget: usize) -> bool {
        let mut submission = self.submission.lock().unwrap();
        let Some(submission) = submission.as_mut() else { return false };
        let job = Job { sequence: submission.next_sequence, stream: stream.0.clone(), payload, reception_ns, budget };
        if submission.sender.send(job).is_err() {
            return false;
        }
        submission.next_sequence += 1;
        true
    }

    /// Counts a message the caller left out on purpose (e.g. over a stream's max rate).
    pub fn skip(&self, stream: &Stream) {
        stream.0.counters.skipped.fetch_add(1, Ordering::Relaxed);
    }

    /// Live counters, per stream and in total.
    pub fn stats(&self) -> Stats {
        if let Some(stats) = self.final_stats.lock().unwrap().clone() {
            return stats;
        }
        self.collect_stats()
    }

    fn collect_stats(&self) -> Stats {
        let mut stats = Stats {
            path: self.shared.options.path.display().to_string(),
            seconds: self.started.elapsed().as_secs_f64(),
            failure: self.shared.failure.lock().unwrap().clone(),
            ..Stats::default()
        };
        for stream in self.shared.streams.lock().unwrap().iter() {
            let counters = &stream.counters;
            let entry = StreamStats {
                name: stream.spec.name.clone(),
                stored_name: stream.stored_name.clone(),
                msg_type: stream.msg_type.clone(),
                received: counters.received.load(Ordering::Relaxed),
                written: counters.written.load(Ordering::Relaxed),
                bytes: counters.bytes.load(Ordering::Relaxed),
                dropped: counters.dropped.load(Ordering::Relaxed),
                skipped: counters.skipped.load(Ordering::Relaxed),
                rejected: counters.rejected.load(Ordering::Relaxed),
                fallback: counters.fallback.load(Ordering::Relaxed),
                last_error: stream.last_error.lock().unwrap().clone(),
            };
            stats.received += entry.received;
            stats.written += entry.written;
            stats.bytes += entry.bytes;
            stats.dropped += entry.dropped;
            stats.skipped += entry.skipped;
            stats.rejected += entry.rejected;
            stats.fallback += entry.fallback;
            stats.streams.push(entry);
        }
        stats
    }

    /// Writes everything queued, finalizes the file (indexes / one self-contained .db) and returns the final counts.
    /// An error means the file could not be finished (the counts are still in `stats()`).
    pub fn finish(&self) -> Result<Stats> {
        drop(self.submission.lock().unwrap().take());
        for worker in self.workers.lock().unwrap().drain(..) {
            let _ = worker.join();
        }
        if let Some(writer) = self.writer.lock().unwrap().take() {
            if writer.join().is_err() {
                self.shared.failure.lock().unwrap().get_or_insert("the writer thread panicked".into());
            }
        }
        self.shared.stopped.store(true, Ordering::SeqCst);
        let stats = self.collect_stats();
        *self.final_stats.lock().unwrap() = Some(stats.clone());
        match &stats.failure {
            Some(failure) => Err(anyhow!("recording {} failed: {failure}", stats.path)),
            None => Ok(stats),
        }
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

fn encode_loop(shared: &Shared, jobs: Receiver<Job>, done: Sender<Done>) {
    for job in jobs {
        let Job { sequence, stream, payload, reception_ns, budget } = job;
        let result = catch_unwind(AssertUnwindSafe(|| process::process(&shared.options, &stream, payload, reception_ns)))
            .unwrap_or_else(|_| Err(anyhow!("encoding panicked")))
            .and_then(|processed| check_times(&shared.options, processed));
        if done.send(Done { sequence, stream, result, budget }).is_err() {
            return;
        }
    }
}

/// MCAP times must be representable (cc's rule: rejected, never clamped); checked before the writer sees them.
fn check_times(options: &Options, processed: process::Processed) -> Result<process::Processed> {
    if options.format != Format::Sqlite {
        for record in &processed.records {
            record.source.mcap_nanos()?;
        }
    }
    Ok(processed)
}

fn writer_loop(shared: &Shared, mut store: Box<dyn store::Store>, done: Receiver<Done>) -> Result<()> {
    let mut pending: BTreeMap<u64, Done> = BTreeMap::new();
    let mut next_sequence = 0u64;
    let mut last_flush = Instant::now();
    let mut failed: Option<anyhow::Error> = None;
    loop {
        let received = done.recv_timeout(Duration::from_millis(100));
        let disconnected = matches!(received, Err(RecvTimeoutError::Disconnected));
        if let Ok(item) = received {
            pending.insert(item.sequence, item);
        }
        while let Some(item) = pending.remove(&next_sequence) {
            next_sequence += 1;
            shared.pending_messages.fetch_sub(1, Ordering::SeqCst);
            shared.pending_bytes.fetch_sub(item.budget, Ordering::SeqCst);
            let counters = &item.stream.counters;
            if failed.is_some() {
                counters.dropped.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            match item.result {
                Ok(processed) => {
                    if let Some(reason) = processed.fallback {
                        counters.fallback.fetch_add(1, Ordering::Relaxed);
                        item.stream.note_error(reason);
                    }
                    match store.write(&processed.records) {
                        Ok(bytes) => {
                            counters.written.fetch_add(processed.records.len() as u64, Ordering::Relaxed);
                            counters.bytes.fetch_add(bytes, Ordering::Relaxed);
                        }
                        Err(error) => {
                            counters.dropped.fetch_add(1, Ordering::Relaxed);
                            shared.stopped.store(true, Ordering::SeqCst);
                            failed = Some(error.context(format!("writing stream {:?}", item.stream.stored_name)));
                        }
                    }
                }
                Err(error) => {
                    counters.rejected.fetch_add(1, Ordering::Relaxed);
                    item.stream.note_error(format!("{error:#}"));
                }
            }
        }
        if failed.is_none() && (disconnected || last_flush.elapsed() >= FLUSH_INTERVAL) {
            if let Err(error) = store.flush() {
                shared.stopped.store(true, Ordering::SeqCst);
                failed = Some(error.context("flushing the recording"));
            }
            last_flush = Instant::now();
        }
        if disconnected {
            break;
        }
    }
    if let Some(error) = failed {
        let _ = store.finish();
        return Err(error);
    }
    if !pending.is_empty() {
        bail!("encoded results ended with a sequence gap");
    }
    // every added stream appears, even one that never got a message
    let streams = shared.streams.lock().unwrap().clone();
    for stream in streams {
        if stream.channels.lock().unwrap().is_empty() {
            process::empty_channel(&shared.options, &stream)?;
        }
        let channels: Vec<Arc<ChannelSpec>> = stream.channels.lock().unwrap().values().cloned().collect();
        for channel in channels {
            store.declare(&channel)?;
        }
    }
    store.flush()?;
    store.finish()
}

pub fn now_nanos() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|age| age.as_nanos() as u64).unwrap_or(0)
}

/// Whether `path` is a finished recording (a killed .db run leaves its write-ahead log beside it).
pub fn is_finished(path: &Path) -> bool {
    !companions(path)[0].exists()
}
