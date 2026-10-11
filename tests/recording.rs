use std::collections::HashMap;

use rust_recorder::lcm::{self, Value};
use rust_recorder::testing::sample_value;
use rust_recorder::{Codec, Format, ImageFormat, Options, Recorder, Recovered, StreamSpec};

fn encode(msg_type: &str, value: &Value) -> Vec<u8> {
    lcm::registry().encode(msg_type, value).unwrap()
}

fn image(encoding: &str, width: usize, height: usize, bytes_per_pixel: usize, data: Option<Vec<u8>>) -> Vec<u8> {
    let mut value = sample_value("sensor_msgs.Image", 1);
    let Value::Struct(fields) = &mut value else { unreachable!() };
    fields[2] = Value::Int(height as i64);
    fields[3] = Value::Int(width as i64);
    fields[4] = Value::Str(encoding.as_bytes().to_vec());
    fields[5] = Value::Int(0);
    let data = data.unwrap_or_else(|| (0..width * height * bytes_per_pixel).map(|index| index as u8).collect());
    fields[6] = Value::Int(if encoding == "jpeg" { 0 } else { (width * bytes_per_pixel) as i64 });
    fields[7] = Value::Bytes(data);
    encode("sensor_msgs.Image", &value)
}

/// topic → [(message encoding, schema name, publish time)]
fn mcap_messages(path: &std::path::Path) -> HashMap<String, Vec<(String, Option<String>, u64)>> {
    let bytes = std::fs::read(path).unwrap();
    let mut seen: HashMap<String, Vec<_>> = HashMap::new();
    for message in mcap::MessageStream::new(&bytes).unwrap() {
        let message = message.unwrap();
        seen.entry(message.channel.topic.clone()).or_default().push((
            message.channel.message_encoding.clone(),
            message.channel.schema.as_ref().map(|schema| schema.name.clone()),
            message.publish_time,
        ));
    }
    seen
}

#[test]
fn ros2_recording_keeps_everything_and_says_how() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("run.mcap");
    let mut options = Options::new(Format::Ros2Mcap, &path);
    options.image_format = if cfg!(feature = "webp") { ImageFormat::Webp } else { ImageFormat::Jpeg };
    let recorder = Recorder::open(options).unwrap();
    let odom = recorder.add_stream(StreamSpec::new("/odom", "nav_msgs.Odometry")).unwrap();
    let camera = recorder.add_stream(StreamSpec::new("/camera", "sensor_msgs.Image")).unwrap();
    let depth = recorder.add_stream(StreamSpec::new("/depth", "sensor_msgs.Image")).unwrap();
    let jpeg_transport = recorder.add_stream(StreamSpec::new("/jpeg", "sensor_msgs.Image")).unwrap();
    let tf = recorder.add_stream(StreamSpec::new("/tf", "tf2_msgs.TFMessage")).unwrap();
    let unknown = recorder.add_stream(StreamSpec::new("/control", "control_msgs.ControlValues")).unwrap();
    let not_lcm = recorder.add_stream(StreamSpec::new("/mystery", "sensor_msgs.ImuInfo")).unwrap();
    let logs = recorder.add_json_stream("/logs", "foxglove.Log", br#"{"type":"object"}"#).unwrap();
    let never = recorder.add_stream(StreamSpec::new("/never", "geometry_msgs.Twist")).unwrap();

    let odometry = sample_value("nav_msgs.Odometry", 3);
    assert!(recorder.record(&odom, &encode("nav_msgs.Odometry", &odometry)));
    recorder.record(&odom, &[9u8; 40]); // garbled: kept as LCM, not dropped
    recorder.record(&camera, &image("rgb8", 8, 4, 3, None));
    recorder.record(&depth, &image("16UC1", 8, 4, 2, None));
    recorder.record(&jpeg_transport, &image("jpeg", 8, 4, 0, Some(vec![0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3])));
    let mut tf_message = sample_value("tf2_msgs.TFMessage", 4);
    if let Value::Struct(fields) = &mut tf_message {
        let Value::Array(transforms) = &mut fields[1] else { unreachable!() };
        let first = transforms[0].clone();
        transforms.push(first);
        fields[0] = Value::Int(transforms.len() as i64);
    }
    let transform_count = match &tf_message.fields().unwrap()[1] {
        Value::Array(transforms) => transforms.len(),
        _ => unreachable!(),
    };
    recorder.record(&tf, &encode("tf2_msgs.TFMessage", &tf_message));
    recorder.record(&unknown, &encode("control_msgs.ControlValues", &sample_value("control_msgs.ControlValues", 5)));
    recorder.record(&not_lcm, b"opaque");
    // a late stream, after messages were written
    let late = recorder.add_stream(StreamSpec::new("/late", "geometry_msgs.Twist")).unwrap();
    recorder.record(&late, &encode("geometry_msgs.Twist", &sample_value("geometry_msgs.Twist", 6)));
    recorder.record_json(&logs, br#"{"message":"hi"}"#.to_vec(), 1_700_000_000_000_000_000).unwrap();
    recorder.skip(&odom);
    let stats = recorder.finish().unwrap();
    let _ = never;

    let seen = mcap_messages(&path);
    let stamp = |value: &Value| {
        let header = &value.fields().unwrap()[0];
        let time = header.fields().unwrap()[1].fields().unwrap();
        time[0].as_i64().unwrap() as u64 * 1_000_000_000 + time[1].as_i64().unwrap() as u64
    };
    assert_eq!(seen["/odom"][0], ("cdr".into(), Some("nav_msgs/msg/Odometry".into()), stamp(&odometry)));
    assert_eq!(seen["/odom"][1].0, "lcm");
    assert_eq!(seen["/camera"][0].1.as_deref(), Some("sensor_msgs/msg/CompressedImage"));
    assert_eq!(seen["/depth"][0].1.as_deref(), Some("sensor_msgs/msg/Image"), "16-bit depth never goes through webp");
    assert_eq!(seen["/jpeg"][0].1.as_deref(), Some("sensor_msgs/msg/CompressedImage"));
    assert_eq!(seen["/tf"].len(), transform_count, "one TFMessage per transform");
    assert_eq!(seen["/control"][0].0, "lcm");
    assert_eq!(seen["/mystery"][0].0, "lcm");
    assert_eq!(seen["/late"][0].1.as_deref(), Some("geometry_msgs/msg/Twist"));
    assert_eq!(seen["/logs"][0], ("json".into(), Some("foxglove.Log".into()), 1_700_000_000_000_000_000));

    let bytes = std::fs::read(&path).unwrap();
    let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
    assert!(summary.channels.values().any(|channel| channel.topic == "/never"), "an empty stream is still declared");
    let by_name: HashMap<_, _> = stats.streams.iter().map(|stream| (stream.name.as_str(), stream)).collect();
    assert_eq!(by_name["/odom"].fallback, 1);
    assert!(by_name["/odom"].last_error.as_deref().unwrap().contains("fingerprint"));
    assert_eq!(by_name["/odom"].skipped, 1);
    assert_eq!(by_name["/tf"].written as usize, transform_count);
    assert_eq!((stats.dropped, stats.rejected), (0, 0));
}

#[test]
fn strict_cdr_refuses_instead_of_keeping_lcm() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("strict.mcap");
    let mut options = Options::new(Format::Ros2Mcap, &path);
    options.strict_cdr = true;
    let recorder = Recorder::open(options).unwrap();
    let control = recorder.add_stream(StreamSpec::new("/control", "control_msgs.ControlValues")).unwrap();
    recorder.record(&control, &encode("control_msgs.ControlValues", &sample_value("control_msgs.ControlValues", 1)));
    let stats = recorder.finish().unwrap();
    assert_eq!((stats.rejected, stats.written), (1, 0));
    assert!(mcap_messages(&path).is_empty());
}

#[test]
fn unrepresentable_times_are_rejected_not_clamped() {
    let directory = tempfile::tempdir().unwrap();
    let mut bad_stamp = sample_value("geometry_msgs.PoseStamped", 2);
    if let Value::Struct(fields) = &mut bad_stamp {
        let Value::Struct(header) = &mut fields[0] else { unreachable!() };
        header[1] = Value::Struct(vec![Value::Int(5), Value::Int(1_500_000_000)]);
    }
    // ROS 2: nanosec ≥ 1e9 cannot be a builtin_interfaces/Time, so the message stays LCM (counted)
    let path = directory.path().join("ros2.mcap");
    let recorder = Recorder::open(Options::new(Format::Ros2Mcap, &path)).unwrap();
    let pose = recorder.add_stream(StreamSpec::new("/pose", "geometry_msgs.PoseStamped")).unwrap();
    recorder.record(&pose, &encode("geometry_msgs.PoseStamped", &bad_stamp));
    let stats = recorder.finish().unwrap();
    assert_eq!(stats.fallback, 1);
    assert_eq!(mcap_messages(&path)["/pose"][0].0, "lcm");

    // JSON source times: MCAP rejects a negative one, SQLite keeps it (dimos' rule)
    let events = |format: Format, path: &std::path::Path| {
        let recorder = Recorder::open(Options::new(format, path)).unwrap();
        let mut spec = StreamSpec::new("events", "std_msgs.String").with_codec(Codec::Json);
        spec.timestamp_field = Some("ts".into());
        let stream = recorder.add_stream(spec).unwrap();
        for document in [r#"{"ts":-12.5}"#, r#"{"ts":12.5}"#, r#"{"ts":"x"}"#] {
            recorder.record(&stream, &encode("std_msgs.String", &Value::Struct(vec![Value::Str(document.as_bytes().to_vec())])));
        }
        recorder.finish().unwrap()
    };
    let mcap_stats = events(Format::DimosMcap, &directory.path().join("events.mcap"));
    assert_eq!((mcap_stats.written, mcap_stats.rejected), (1, 2));
    let db = directory.path().join("events.db");
    let sqlite_stats = events(Format::Sqlite, &db);
    assert_eq!((sqlite_stats.written, sqlite_stats.rejected), (2, 1));
    let connection = rusqlite::Connection::open(&db).unwrap();
    let times: Vec<f64> = connection.prepare("SELECT ts FROM events ORDER BY id").unwrap().query_map([], |row| row.get(0)).unwrap().map(Result::unwrap).collect();
    assert_eq!(times, vec![-12.5, 12.5]);
}

#[test]
fn a_full_queue_drops_and_counts() {
    let directory = tempfile::tempdir().unwrap();
    let mut options = Options::new(Format::DimosMcap, directory.path().join("full.mcap"));
    options.queue_messages = 0;
    let recorder = Recorder::open(options).unwrap();
    let odom = recorder.add_stream(StreamSpec::new("odom", "nav_msgs.Odometry")).unwrap();
    for _ in 0..5 {
        assert!(!recorder.record(&odom, &encode("nav_msgs.Odometry", &sample_value("nav_msgs.Odometry", 1))));
    }
    let stats = recorder.finish().unwrap();
    assert_eq!((stats.received, stats.dropped, stats.written), (5, 5, 0));
}

#[test]
fn dimos_layout_picks_codecs_and_names_like_dimos() {
    let directory = tempfile::tempdir().unwrap();
    let db = directory.path().join("run.db");
    let recorder = Recorder::open(Options::new(Format::Sqlite, &db)).unwrap();
    let color = recorder.add_stream(StreamSpec::new("/camera/color", "sensor_msgs.Image")).unwrap();
    let depth = recorder.add_stream(StreamSpec::new("/camera/depth", "sensor_msgs.Image")).unwrap();
    let odom = recorder.add_stream(StreamSpec::new("/odom", "dimos.msgs.nav_msgs.Odometry.Odometry")).unwrap();
    let odom_pose = recorder.add_stream(StreamSpec::new("/odom", "geometry_msgs.PoseStamped")).unwrap();
    recorder.record(&color, &image("bgr8", 8, 4, 3, None));
    recorder.record(&depth, &image("32FC1", 8, 4, 4, None));
    recorder.record(&odom, &encode("nav_msgs.Odometry", &sample_value("nav_msgs.Odometry", 2)));
    recorder.record(&odom_pose, &encode("geometry_msgs.PoseStamped", &sample_value("geometry_msgs.PoseStamped", 2)));
    recorder.finish().unwrap();
    assert!(rust_recorder::is_finished(&db));
    let connection = rusqlite::Connection::open(&db).unwrap();
    let registered: HashMap<String, String> = connection
        .prepare("SELECT name, json_extract(config, '$.codec_id') FROM _streams")
        .unwrap()
        .query_map([], |row| Ok((row.get(0)?, row.get(1)?)))
        .unwrap()
        .map(Result::unwrap)
        .collect();
    assert_eq!(registered["camera_color"], "jpeg");
    assert_eq!(registered["camera_depth"], "lz4+lcm");
    assert_eq!(registered["odom"], "lcm");
    assert_eq!(registered["odom_PoseStamped"], "lcm");
}

/// Runs only as the child of `a_killed_run_is_recovered`: records, then waits to be killed.
#[test]
fn killed_run_child() {
    let Ok(path) = std::env::var("RUST_RECORDER_KILL_CHILD") else { return };
    let path = std::path::PathBuf::from(path);
    let format = if path.extension().unwrap() == "db" { Format::Sqlite } else { Format::Ros2Mcap };
    let recorder = Recorder::open(Options::new(format, &path)).unwrap();
    let odom = recorder.add_stream(StreamSpec::new("/odom", "nav_msgs.Odometry")).unwrap();
    for seed in 0..10 {
        recorder.record(&odom, &encode("nav_msgs.Odometry", &sample_value("nav_msgs.Odometry", seed)));
    }
    std::fs::write(path.with_extension("ready"), b"").unwrap();
    std::thread::sleep(std::time::Duration::from_secs(60));
}

#[test]
fn a_killed_run_is_recovered() {
    let directory = tempfile::tempdir().unwrap();
    for format in [Format::Sqlite, Format::Ros2Mcap] {
        let path = directory.path().join(format!("killed.{}", format.extension()));
        let mut child = std::process::Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "killed_run_child", "--nocapture", "--test-threads=1"])
            .env("RUST_RECORDER_KILL_CHILD", &path)
            .stdout(std::process::Stdio::null())
            .spawn()
            .unwrap();
        let ready = path.with_extension("ready");
        while !ready.exists() {
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        // the writer commits (SQLite) or closes a chunk (MCAP) at least once a second
        std::thread::sleep(std::time::Duration::from_millis(1600));
        child.kill().unwrap();
        child.wait().unwrap();
        assert!(!rust_recorder::is_finished(&path) || format != Format::Sqlite);
        assert_eq!(rust_recorder::recover(&path).unwrap(), Recovered::Repaired { messages: 10 }, "{format:?}");
        assert_eq!(rust_recorder::recover(&path).unwrap(), Recovered::Finished);
    }
}
