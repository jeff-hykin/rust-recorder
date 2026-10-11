use std::sync::Arc;

use rust_recorder::lcm;
use rust_recorder::testing::sample_value;
use rust_recorder::{Format, Options, Recorder};

#[tokio::test(flavor = "multi_thread")]
async fn records_a_dimos_key_from_zenoh() {
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    config.insert_json5("listen/endpoints", r#"["tcp/127.0.0.1:0"]"#).unwrap();
    let session = zenoh::open(config).await.unwrap();
    let directory = std::env::temp_dir().join(format!("rust-recorder-zenoh-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("loopback.mcap");
    let recorder = Arc::new(Recorder::open(Options::new(Format::Ros2Mcap, &path)).unwrap());
    let key = "dimos/odom/nav_msgs.Odometry";
    let (stream, subscriber) = rust_recorder_zenoh_1_10::zenoh_recording::record_dimos_key(&session, recorder.clone(), key, || true).await.unwrap();
    assert_eq!(stream.stored_name(), "/odom");
    let payload = lcm::registry().encode("nav_msgs.Odometry", &sample_value("nav_msgs.Odometry", 1)).unwrap();
    for _ in 0..5 {
        session.put(key, payload.clone()).await.unwrap();
    }
    for _ in 0..100 {
        if recorder.stats().received == 5 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;
    }
    subscriber.undeclare().await.unwrap();
    let stats = recorder.finish().unwrap();
    assert_eq!((stats.received, stats.written), (5, 5));
    let bytes = std::fs::read(&path).unwrap();
    let messages: Vec<_> = mcap::MessageStream::new(&bytes).unwrap().map(Result::unwrap).collect();
    assert_eq!(messages.len(), 5);
    assert_eq!(messages[0].channel.schema.as_ref().unwrap().name, "nav_msgs/msg/Odometry");
    std::fs::remove_dir_all(&directory).unwrap();
    session.close().await.unwrap();
}
