//! The writers: a dimos SqliteStore `.db` (the tables dimos' recorder writes plus the `_streams` registry dimos'
//! SqliteStore reads) and MCAP (dimos or ros2 profile). Both commit/flush at least once a second, so a killed run
//! loses at most that second, and `recover` turns what a killed run left into a finished file.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result};
use rusqlite::{params, Connection};

use crate::process::{ChannelSpec, Record};
use crate::{Compression, Format, Options};

pub(crate) trait Store: Send {
    /// Declares a channel (also for streams that never got a message).
    fn declare(&mut self, channel: &Arc<ChannelSpec>) -> Result<()>;
    /// Appends records in arrival order; returns the bytes written.
    fn write(&mut self, records: &[Record]) -> Result<u64>;
    /// Makes everything so far durable.
    fn flush(&mut self) -> Result<()>;
    fn finish(&mut self) -> Result<()>;
}

pub(crate) fn open(options: &Options) -> Result<Box<dyn Store>> {
    if let Some(parent) = options.path.parent().filter(|parent| !parent.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).with_context(|| format!("could not create {}", parent.display()))?;
    }
    match options.format {
        Format::Sqlite => Ok(Box::new(SqliteStore::open(&options.path)?)),
        Format::DimosMcap | Format::Ros2Mcap => Ok(Box::new(McapStore::open(options)?)),
    }
}

fn channel_key(channel: &ChannelSpec) -> String {
    format!("{}\u{0}{}\u{0}{}", channel.topic, channel.message_encoding, channel.schema.as_ref().map(|(name, ..)| name.as_str()).unwrap_or(""))
}

pub(crate) struct SqliteStore {
    connection: Connection,
    declared: HashMap<String, bool>,
    in_transaction: bool,
}

impl SqliteStore {
    fn open(path: &Path) -> Result<Self> {
        let connection = Connection::open(path).with_context(|| format!("could not open {}", path.display()))?;
        connection.pragma_update(None, "journal_mode", "WAL")?;
        connection.pragma_update(None, "synchronous", "NORMAL")?;
        connection.execute_batch("CREATE TABLE IF NOT EXISTS _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")?;
        Ok(SqliteStore { connection, declared: HashMap::new(), in_transaction: false })
    }

    fn begin(&mut self) -> Result<()> {
        if !self.in_transaction {
            self.connection.execute_batch("BEGIN")?;
            self.in_transaction = true;
        }
        Ok(())
    }
}

/// The registry entry, byte for byte what dimos' SqliteStore._serialize_backend stores (json.dumps of the dict).
pub(crate) fn registry_config(payload_type: &str, codec_id: &str) -> String {
    format!(
        r#"{{"payload_module": {}, "codec_id": {}, "eager_blobs": false, "page_size": 256, "blob_store": {{"class": "dimos.memory.blobstore.sqlite.SqliteBlobStore", "config": {{"path": null}}}}, "vector_store": {{"class": "dimos.memory.vectorstore.sqlite.SqliteVectorStore", "config": {{"path": null}}}}, "notifier": {{"class": "dimos.memory.notifier.subject.SubjectNotifier", "config": {{}}}}}}"#,
        serde_json::to_string(payload_type).unwrap(),
        serde_json::to_string(codec_id).unwrap()
    )
}

impl Store for SqliteStore {
    fn declare(&mut self, channel: &Arc<ChannelSpec>) -> Result<()> {
        if self.declared.contains_key(&channel.topic) {
            return Ok(());
        }
        let sqlite = channel.sqlite.as_ref().context("SQLite stream without a registration")?;
        let name = &channel.topic;
        crate::validate_identifier(name)?;
        self.begin()?;
        // the tables dimos' recorder (and cc's native recorder) creates
        self.connection.execute_batch(&format!(
            r#"
            CREATE TABLE IF NOT EXISTS "{name}" (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ts REAL NOT NULL,
                value NUMERIC,
                pose_x REAL, pose_y REAL, pose_z REAL,
                pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
                tags BLOB DEFAULT (jsonb('{{}}'))
            );
            CREATE TABLE IF NOT EXISTS "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL);
            CREATE VIRTUAL TABLE IF NOT EXISTS "{name}_rtree" USING rtree(id, x_min, x_max, y_min, y_max, z_min, z_max);
            "#
        ))?;
        if !sqlite.is_tf {
            self.connection.execute_batch(&format!(r#"CREATE INDEX IF NOT EXISTS "{name}_tag_reception_ts" ON "{name}"(json_extract(tags, '$.reception_ts'))"#))?;
        }
        self.connection.execute(
            "INSERT OR REPLACE INTO _streams (name, config) VALUES (?1, ?2)",
            params![name, registry_config(&sqlite.payload_type, &sqlite.codec_id)],
        )?;
        self.declared.insert(name.clone(), sqlite.is_tf);
        Ok(())
    }

    fn write(&mut self, records: &[Record]) -> Result<u64> {
        let mut bytes = 0;
        for record in records {
            self.declare(&record.channel)?;
            self.begin()?;
            let name = &record.channel.topic;
            let source = record.source.seconds();
            if record.channel.sqlite.as_ref().is_some_and(|sqlite| sqlite.is_tf) {
                self.connection.prepare_cached(&format!(r#"INSERT INTO "{name}" (ts) VALUES (?1)"#))?.execute(params![source])?;
            } else {
                let tags = serde_json::json!({ "reception_ts": record.reception_ns as f64 / 1e9 }).to_string();
                self.connection.prepare_cached(&format!(r#"INSERT INTO "{name}" (ts, tags) VALUES (?1, jsonb(?2))"#))?.execute(params![source, tags])?;
            }
            let id = self.connection.last_insert_rowid();
            self.connection.prepare_cached(&format!(r#"INSERT INTO "{name}_blob" (id, data) VALUES (?1, ?2)"#))?.execute(params![id, record.data])?;
            bytes += record.data.len() as u64;
        }
        Ok(bytes)
    }

    fn flush(&mut self) -> Result<()> {
        if self.in_transaction {
            self.connection.execute_batch("COMMIT")?;
            self.in_transaction = false;
        }
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        self.flush()?;
        // one self-contained file, like a finished dimos recording
        self.connection.execute_batch("PRAGMA wal_checkpoint(TRUNCATE); PRAGMA journal_mode = DELETE;")?;
        Ok(())
    }
}

pub(crate) struct McapStore {
    writer: Option<mcap::Writer<BufWriter<File>>>,
    channels: HashMap<String, (u16, u32)>,
    schemas: HashMap<(String, Vec<u8>), u16>,
}

impl McapStore {
    fn open(options: &Options) -> Result<Self> {
        let file = File::create(&options.path).with_context(|| format!("could not create {}", options.path.display()))?;
        let profile = if options.format == Format::Ros2Mcap { "ros2" } else { "dimos" };
        let writer = mcap::WriteOptions::new()
            .profile(profile)
            .library(concat!("rust-recorder ", env!("CARGO_PKG_VERSION")))
            .compression(match options.compression {
                Compression::None => None,
                Compression::Lz4 => Some(mcap::Compression::Lz4),
                Compression::Zstd => Some(mcap::Compression::Zstd),
            })
            .create(BufWriter::new(file))?;
        Ok(McapStore { writer: Some(writer), channels: HashMap::new(), schemas: HashMap::new() })
    }

    fn channel_id(&mut self, channel: &ChannelSpec) -> Result<String> {
        let key = channel_key(channel);
        if self.channels.contains_key(&key) {
            return Ok(key);
        }
        let writer = self.writer.as_mut().context("MCAP writer already finished")?;
        let schema_id = match &channel.schema {
            Some((name, encoding, data)) => {
                let schema_key = (name.clone(), data.to_vec());
                match self.schemas.get(&schema_key) {
                    Some(id) => *id,
                    None => {
                        let id = writer.add_schema(name, encoding, data)?;
                        self.schemas.insert(schema_key, id);
                        id
                    }
                }
            }
            None => 0,
        };
        let id = writer.add_channel(schema_id, &channel.topic, &channel.message_encoding, &channel.metadata)?;
        self.channels.insert(key.clone(), (id, 0));
        Ok(key)
    }
}

impl Store for McapStore {
    fn declare(&mut self, channel: &Arc<ChannelSpec>) -> Result<()> {
        self.channel_id(channel).map(|_| ())
    }

    fn write(&mut self, records: &[Record]) -> Result<u64> {
        let mut bytes = 0;
        for record in records {
            // validated before anything is written, so a bad time leaves no partial record
            let publish_time = record.source.mcap_nanos().with_context(|| format!("stream {:?}", record.channel.topic))?;
            let key = self.channel_id(&record.channel)?;
            let (channel_id, sequence) = self.channels.get_mut(&key).unwrap();
            let header = mcap::records::MessageHeader { channel_id: *channel_id, sequence: *sequence, log_time: record.reception_ns, publish_time };
            *sequence = sequence.wrapping_add(1);
            self.writer.as_mut().context("MCAP writer already finished")?.write_to_known_channel(&header, &record.data)?;
            bytes += record.data.len() as u64;
        }
        Ok(bytes)
    }

    fn flush(&mut self) -> Result<()> {
        if let Some(writer) = self.writer.as_mut() {
            writer.flush()?;
        }
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        if let Some(mut writer) = self.writer.take() {
            writer.finish()?;
        }
        Ok(())
    }
}

/// What `recover` found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Recovered {
    /// already a finished file
    Finished,
    /// a killed run's file, now finished with this many messages
    Repaired { messages: u64 },
}

/// Turns what a killed recording left into a finished file: folds a .db's write-ahead log into it, or rewrites an
/// MCAP that has no summary (keeping every complete message). Safe to run on a finished file.
pub fn recover(path: &Path) -> Result<Recovered> {
    let is_mcap = path.extension().is_some_and(|end| end == "mcap");
    if !is_mcap {
        let wal = with_suffix(path, "-wal");
        if !wal.exists() {
            return Ok(Recovered::Finished);
        }
        let connection = Connection::open(path).with_context(|| format!("could not open {}", path.display()))?;
        let names: Vec<String> = connection.prepare("SELECT name FROM _streams")?.query_map([], |row| row.get(0))?.collect::<Result<_, _>>()?;
        let mut messages = 0u64;
        for name in names {
            messages += connection.query_row(&format!(r#"SELECT count(*) FROM "{name}""#), [], |row| row.get::<_, i64>(0))? as u64;
        }
        connection.execute_batch("PRAGMA wal_checkpoint(TRUNCATE); PRAGMA journal_mode = DELETE;")?;
        return Ok(Recovered::Repaired { messages });
    }
    let bytes = std::fs::read(path).with_context(|| format!("could not read {}", path.display()))?;
    if mcap::Summary::read(&bytes).ok().flatten().is_some() {
        return Ok(Recovered::Finished);
    }
    let header = mcap::read::LinearReader::new_with_options(&bytes, mcap::read::Options::IgnoreEndMagic.into())?
        .find_map(|record| match record {
            Ok(mcap::records::Record::Header(header)) => Some(header),
            _ => None,
        })
        .context("not an MCAP file (no header)")?;
    let temporary = with_suffix(path, ".recovering");
    let mut writer = mcap::WriteOptions::new()
        .profile(header.profile.clone())
        .library(header.library.clone())
        .compression(Some(mcap::Compression::Zstd))
        .create(BufWriter::new(File::create(&temporary)?))?;
    let mut channels: HashMap<u16, u16> = HashMap::new();
    let mut schemas: HashMap<u16, u16> = HashMap::new();
    let mut messages = 0u64;
    for message in mcap::MessageStream::new_with_options(&bytes, mcap::read::Options::IgnoreEndMagic.into())? {
        // a killed run ends mid-record: everything before it is kept
        let Ok(message) = message else { break };
        let channel_id = match channels.get(&message.channel.id) {
            Some(id) => *id,
            None => {
                let schema_id = match &message.channel.schema {
                    Some(schema) => match schemas.get(&schema.id) {
                        Some(id) => *id,
                        None => {
                            let id = writer.add_schema(&schema.name, &schema.encoding, &schema.data)?;
                            schemas.insert(schema.id, id);
                            id
                        }
                    },
                    None => 0,
                };
                let id = writer.add_channel(schema_id, &message.channel.topic, &message.channel.message_encoding, &message.channel.metadata)?;
                channels.insert(message.channel.id, id);
                id
            }
        };
        writer.write_to_known_channel(
            &mcap::records::MessageHeader { channel_id, sequence: message.sequence, log_time: message.log_time, publish_time: message.publish_time },
            &message.data,
        )?;
        messages += 1;
    }
    writer.finish()?;
    drop(writer);
    std::fs::rename(&temporary, path)?;
    Ok(Recovered::Repaired { messages })
}

fn with_suffix(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.as_os_str().to_owned();
    name.push(suffix);
    PathBuf::from(name)
}

/// A .db's write-ahead log and shared memory, to move or delete along with it.
pub fn companions(path: &Path) -> [PathBuf; 2] {
    [with_suffix(path, "-wal"), with_suffix(path, "-shm")]
}
