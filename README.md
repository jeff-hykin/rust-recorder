# rust-recorder

A standalone Rust library that records dimos streams to:

- a **dimos SqliteStore `.db`**,
- a **dimos-layout MCAP**,
- a **ROS 2 MCAP** (CDR with full `ros2msg` schemas) that opens directly in Foxglove.

It is an isolated version of the native recorder TomCC7 (cc) wrote for dimos (`dimos/experimental/memory/rust`), plus full LCM → ROS 2 CDR transcoding.

You hand it bytes (dimos LCM) and a type name; it has no dimos-module, transport or zenoh dependency. So it works the same in the dimos runtime (zenoh 1.10), in Desktop apps (zenoh 1.6), or in anything else. Dependencies are lean: mcap, rusqlite (bundled SQLite), lz4_flex, a pure-Rust JPEG encoder, and optional PNG/WebP/JPEG-XL encoders. No cmake and no turbojpeg.

## Usage

```toml
[dependencies]
rust-recorder = { git = "https://github.com/jeff-hykin/rust-recorder", tag = "v0.1.0" }
```

```rust
use std::sync::Arc;
use rust_recorder::{Format, ImageFormat, Options, Recorder, StreamSpec};

let mut options = Options::new(Format::Ros2Mcap, "run.mcap"); // or Format::Sqlite ("run.db"), Format::DimosMcap
options.image_format = ImageFormat::Jpeg; // ROS 2 only: raw frames → CompressedImage (depth stays raw)
let recorder = Arc::new(Recorder::open(options)?);

let odom = recorder.add_stream(StreamSpec::new("/odom", "nav_msgs.Odometry"))?; // any time, also mid-recording
recorder.record(&odom, &lcm_bytes); // never blocks; false = dropped (counted)

let logs = recorder.add_json_stream("/dimos/logs/run", "foxglove.Log", LOG_JSON_SCHEMA)?;
recorder.record_json(&logs, json_bytes, time_ns)?; // never dropped

println!("{:?}", recorder.stats()); // live per-stream counts: written, dropped, skipped, rejected, fallback, last_error
let stats = recorder.finish()?; // indexes written, .db folded into one file
```

**zenoh, any 1.x version.** `rust_recorder::zenoh_recorder!(zenoh);` expands against *your* zenoh crate, so the crate never depends on zenoh itself. It gives you `zenoh_recording::record_dimos_key(&session, recorder, "dimos/odom/nav_msgs.Odometry", admit)` and `record_key(…)`; `admit` is a per-sample hook, e.g. for a max rate. CI builds and runs a loopback test against zenoh `=1.6.2` (Desktop's gateway pin) and `=1.10.1` (dimos' pin). Cargo can't keep two semver-compatible `=` pins of one crate as features (the lockfile resolution fails), so a macro is the way to support both.

**Crash recovery.** SQLite commits and MCAP closes a chunk at least once a second, so a killed run loses at most about that second. `rust_recorder::recover(path)` turns what it left into a finished file: it folds the `.db` write-ahead log back in, or rewrites the MCAP prefix with a summary.

## Formats

| | `Format::Sqlite` | `Format::DimosMcap` | `Format::Ros2Mcap` |
|---|---|---|---|
| file | `.db`, one file when finished (WAL folded in) | `.mcap`, profile `dimos` | `.mcap`, profile `ros2` |
| per stream | tables `<name>`, `<name>_blob`, `<name>_rtree`, plus the `_streams` registry row | one channel, topic = stream name | channel topic = stream name as given (e.g. `/odom`) |
| payload | the dimos storage codec: `lcm`, `jpeg`, `lz4+lcm` or `json` | the same codec as the message encoding | `cdr` with a `ros2msg` schema (`package/msg/Type`, full text with dependencies) |
| source time | `ts` = header stamp (reception time when `sec ≤ 0`) | `publish_time` = the same | `publish_time` = header stamp, exact (zero stays zero) |
| reception time | `tags.reception_ts` | `log_time` | `log_time` |
| TF | one row per transform | one message per transform | one `TFMessage` per transform |
| images | `jpeg` by default (quality 50, 4:2:2, dimos' JpegCodec); frames JPEG can't hold (float depth) go lossless `lz4+lcm` | same | raw `sensor_msgs/Image` by default; `image_format` gives a `CompressedImage` (jpeg, png, webp, jxl) and keeps 16-bit/float depth raw; JPEG-transport frames (`encoding: jpeg`, `step: 0`) become a `CompressedImage` with the original bytes |
| unknown / invalid | stored as-is with reception time / rejected + counted | same | raw LCM channel (`message_encoding: lcm`, `lcm_type` metadata) + counted as `fallback`; `strict_cdr` rejects instead |
| metadata | — | `dimos.payload_type`, `dimos.stream_name`, `dimos.port`, `dimos.observation_time=publish_time` | same keys |

Stream names in dimos stores become SQL identifiers (`/camera/color` → `camera_color`). If a second type arrives on the same name it gets `<name>_<Type>`, the way dimos' McapStore names it.

## ROS 2 CDR coverage

Transcoding is driven by definitions, not hand-written per type:

- the vendored dimos-lcm `.lcm` files (`msgs/lcm`) drive a generic LCM decoder; fingerprints are computed the way lcm-gen does and checked on every message;
- the pinned ROS 2 Jazzy `.msg` files (`msgs/ros2`) drive the CDR encoder and the embedded schema text, so the bytes and the schema always come from the same definition.

Fields are matched by name:

- matching ignores case (dimos-lcm's ROS 1 `CameraInfo.D/K/R/P` → ROS 2 `d/k/r/p`);
- `Time.nsec` → `nanosec`;
- `Header.seq` is dropped;
- ROS 1 `std_msgs/Time` and `Duration` become `builtin_interfaces`.

A type is recorded as CDR only if the mapping is **lossless**: every LCM field has a place in the ROS 2 type, with compatible shapes and widths. Otherwise its messages stay raw LCM. A few ROS 2 types have fields LCM lacks; those fields are written as zero and the table lists them. `cargo run --example coverage` regenerates the tables below.

The types dimos itself publishes (`dimos/msgs/**`):

| dimos type | ROS 2 type | recorded as |
|---|---|---|
| `control_msgs.ControlValues` | — | raw LCM: no ROS 2 counterpart (control_msgs/msg/ControlValues) |
| `foxglove_msgs.CompressedVideo` | `foxglove_msgs/msg/CompressedVideo` | CDR |
| `geometry_msgs.Point` | `geometry_msgs/msg/Point` | CDR |
| `geometry_msgs.PointStamped` | `geometry_msgs/msg/PointStamped` | CDR |
| `geometry_msgs.Pose` | `geometry_msgs/msg/Pose` | CDR |
| `geometry_msgs.PoseArray` | `geometry_msgs/msg/PoseArray` | CDR |
| `geometry_msgs.PoseStamped` | `geometry_msgs/msg/PoseStamped` | CDR |
| `geometry_msgs.PoseWithCovariance` | `geometry_msgs/msg/PoseWithCovariance` | CDR |
| `geometry_msgs.PoseWithCovarianceStamped` | `geometry_msgs/msg/PoseWithCovarianceStamped` | CDR |
| `geometry_msgs.Quaternion` | `geometry_msgs/msg/Quaternion` | CDR |
| `geometry_msgs.Transform` | `geometry_msgs/msg/Transform` | CDR |
| `geometry_msgs.Twist` | `geometry_msgs/msg/Twist` | CDR |
| `geometry_msgs.TwistStamped` | `geometry_msgs/msg/TwistStamped` | CDR |
| `geometry_msgs.TwistWithCovariance` | `geometry_msgs/msg/TwistWithCovariance` | CDR |
| `geometry_msgs.TwistWithCovarianceStamped` | `geometry_msgs/msg/TwistWithCovarianceStamped` | CDR |
| `geometry_msgs.Vector3` | `geometry_msgs/msg/Vector3` | CDR |
| `geometry_msgs.Wrench` | `geometry_msgs/msg/Wrench` | CDR |
| `geometry_msgs.WrenchStamped` | `geometry_msgs/msg/WrenchStamped` | CDR |
| `manipulation_msgs.GraspCandidate` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `manipulation_msgs.GraspCandidateArray` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `nav_msgs.LineSegments3D` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `nav_msgs.OccupancyGrid` | `nav_msgs/msg/OccupancyGrid` | CDR |
| `nav_msgs.Odometry` | `nav_msgs/msg/Odometry` | CDR |
| `nav_msgs.Path` | `nav_msgs/msg/Path` | CDR |
| `sensor_msgs.BatteryState` | `sensor_msgs/msg/BatteryState` | CDR |
| `sensor_msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` | CDR |
| `sensor_msgs.CompressedImage` | `sensor_msgs/msg/CompressedImage` | CDR |
| `sensor_msgs.Image` | `sensor_msgs/msg/Image` | CDR |
| `sensor_msgs.Imu` | `sensor_msgs/msg/Imu` | CDR |
| `sensor_msgs.ImuInfo` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `sensor_msgs.JointCommand` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `sensor_msgs.JointState` | `sensor_msgs/msg/JointState` | CDR |
| `sensor_msgs.Joy` | `sensor_msgs/msg/Joy` | CDR |
| `sensor_msgs.MotorCommandArray` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `sensor_msgs.PointCloud2` | `sensor_msgs/msg/PointCloud2` | CDR |
| `sensor_msgs.RobotState` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `sim_msgs.Contacts` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `std_msgs.Bool` | `std_msgs/msg/Bool` | CDR |
| `std_msgs.Float32` | `std_msgs/msg/Float32` | CDR |
| `std_msgs.Header` | `std_msgs/msg/Header` | CDR |
| `std_msgs.Int32` | `std_msgs/msg/Int32` | CDR |
| `std_msgs.Int8` | `std_msgs/msg/Int8` | CDR |
| `std_msgs.String` | `std_msgs/msg/String` | CDR |
| `std_msgs.UInt32` | `std_msgs/msg/UInt32` | CDR |
| `tf2_msgs.TFMessage` | `tf2_msgs/msg/TFMessage` | CDR |
| `trajectory_msgs.JointTrajectory` | `trajectory_msgs/msg/JointTrajectory` | CDR |
| `trajectory_msgs.TrajectoryPoint` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `trajectory_msgs.TrajectoryStatus` | — | raw LCM (dimos-only type, no dimos-lcm definition) |
| `vision_msgs.BoundingBox2DArray` | `vision_msgs/msg/BoundingBox2DArray` | CDR |
| `vision_msgs.BoundingBox3DArray` | `vision_msgs/msg/BoundingBox3DArray` | CDR |
| `vision_msgs.Detection2D` | `vision_msgs/msg/Detection2D` | CDR |
| `vision_msgs.Detection2DArray` | `vision_msgs/msg/Detection2DArray` | CDR |
| `vision_msgs.Detection3D` | `vision_msgs/msg/Detection3D` | CDR |
| `vision_msgs.Detection3DArray` | `vision_msgs/msg/Detection3DArray` | CDR |
| `visualization_msgs.EntityMarkers` | — | raw LCM (dimos-only type, no dimos-lcm definition) |

<details><summary>Every dimos-lcm type (174)</summary>

| dimos-lcm type | ROS 2 type | recorded as |
|---|---|---|
| `actionlib_msgs.GoalID` | — | raw LCM: no ROS 2 counterpart (actionlib_msgs/msg/GoalID) |
| `actionlib_msgs.GoalStatus` | — | raw LCM: no ROS 2 counterpart (actionlib_msgs/msg/GoalStatus) |
| `actionlib_msgs.GoalStatusArray` | — | raw LCM: no ROS 2 counterpart (actionlib_msgs/msg/GoalStatusArray) |
| `builtin_interfaces.Duration` | `builtin_interfaces/msg/Duration` | CDR |
| `builtin_interfaces.Time` | `builtin_interfaces/msg/Time` | CDR |
| `control_msgs.ControlValues` | — | raw LCM: no ROS 2 counterpart (control_msgs/msg/ControlValues) |
| `diagnostic_msgs.DiagnosticArray` | `diagnostic_msgs/msg/DiagnosticArray` | CDR |
| `diagnostic_msgs.DiagnosticStatus` | `diagnostic_msgs/msg/DiagnosticStatus` | CDR |
| `diagnostic_msgs.KeyValue` | `diagnostic_msgs/msg/KeyValue` | CDR |
| `foxglove_msgs.ArrowPrimitive` | `foxglove_msgs/msg/ArrowPrimitive` | CDR |
| `foxglove_msgs.CameraCalibration` | `foxglove_msgs/msg/CameraCalibration` | CDR |
| `foxglove_msgs.CircleAnnotation` | `foxglove_msgs/msg/CircleAnnotation` | CDR (zero: metadata) |
| `foxglove_msgs.Color` | `foxglove_msgs/msg/Color` | CDR |
| `foxglove_msgs.CompressedImage` | `foxglove_msgs/msg/CompressedImage` | CDR |
| `foxglove_msgs.CompressedVideo` | `foxglove_msgs/msg/CompressedVideo` | CDR |
| `foxglove_msgs.CubePrimitive` | `foxglove_msgs/msg/CubePrimitive` | CDR |
| `foxglove_msgs.CylinderPrimitive` | `foxglove_msgs/msg/CylinderPrimitive` | CDR |
| `foxglove_msgs.FrameTransform` | `foxglove_msgs/msg/FrameTransform` | CDR |
| `foxglove_msgs.FrameTransforms` | `foxglove_msgs/msg/FrameTransforms` | CDR |
| `foxglove_msgs.GeoJSON` | `foxglove_msgs/msg/GeoJSON` | CDR |
| `foxglove_msgs.Grid` | `foxglove_msgs/msg/Grid` | CDR |
| `foxglove_msgs.ImageAnnotations` | `foxglove_msgs/msg/ImageAnnotations` | CDR (zero: timestamp, metadata) |
| `foxglove_msgs.KeyValuePair` | `foxglove_msgs/msg/KeyValuePair` | CDR |
| `foxglove_msgs.LaserScan` | `foxglove_msgs/msg/LaserScan` | CDR |
| `foxglove_msgs.LinePrimitive` | `foxglove_msgs/msg/LinePrimitive` | CDR |
| `foxglove_msgs.LocationFix` | `foxglove_msgs/msg/LocationFix` | CDR (zero: heading, velocity, color, metadata) |
| `foxglove_msgs.Log` | `foxglove_msgs/msg/Log` | CDR |
| `foxglove_msgs.ModelPrimitive` | `foxglove_msgs/msg/ModelPrimitive` | CDR |
| `foxglove_msgs.PackedElementField` | `foxglove_msgs/msg/PackedElementField` | CDR |
| `foxglove_msgs.Point2` | `foxglove_msgs/msg/Point2` | CDR |
| `foxglove_msgs.PointCloud` | `foxglove_msgs/msg/PointCloud` | CDR |
| `foxglove_msgs.PointsAnnotation` | `foxglove_msgs/msg/PointsAnnotation` | CDR (zero: metadata) |
| `foxglove_msgs.PoseInFrame` | `foxglove_msgs/msg/PoseInFrame` | CDR |
| `foxglove_msgs.PosesInFrame` | `foxglove_msgs/msg/PosesInFrame` | CDR |
| `foxglove_msgs.RawAudio` | `foxglove_msgs/msg/RawAudio` | CDR |
| `foxglove_msgs.RawImage` | `foxglove_msgs/msg/RawImage` | CDR |
| `foxglove_msgs.SceneEntity` | `foxglove_msgs/msg/SceneEntity` | CDR |
| `foxglove_msgs.SceneEntityDeletion` | `foxglove_msgs/msg/SceneEntityDeletion` | CDR |
| `foxglove_msgs.SceneUpdate` | `foxglove_msgs/msg/SceneUpdate` | CDR |
| `foxglove_msgs.SpherePrimitive` | `foxglove_msgs/msg/SpherePrimitive` | CDR |
| `foxglove_msgs.TextAnnotation` | `foxglove_msgs/msg/TextAnnotation` | CDR (zero: metadata) |
| `foxglove_msgs.TextPrimitive` | `foxglove_msgs/msg/TextPrimitive` | CDR |
| `foxglove_msgs.TriangleListPrimitive` | `foxglove_msgs/msg/TriangleListPrimitive` | CDR |
| `foxglove_msgs.Vector2` | `foxglove_msgs/msg/Vector2` | CDR |
| `geometry_msgs.Accel` | `geometry_msgs/msg/Accel` | CDR |
| `geometry_msgs.AccelStamped` | `geometry_msgs/msg/AccelStamped` | CDR |
| `geometry_msgs.AccelWithCovariance` | `geometry_msgs/msg/AccelWithCovariance` | CDR |
| `geometry_msgs.AccelWithCovarianceStamped` | `geometry_msgs/msg/AccelWithCovarianceStamped` | CDR |
| `geometry_msgs.Inertia` | `geometry_msgs/msg/Inertia` | CDR |
| `geometry_msgs.InertiaStamped` | `geometry_msgs/msg/InertiaStamped` | CDR |
| `geometry_msgs.Point` | `geometry_msgs/msg/Point` | CDR |
| `geometry_msgs.Point32` | `geometry_msgs/msg/Point32` | CDR |
| `geometry_msgs.PointStamped` | `geometry_msgs/msg/PointStamped` | CDR |
| `geometry_msgs.Polygon` | `geometry_msgs/msg/Polygon` | CDR |
| `geometry_msgs.PolygonStamped` | `geometry_msgs/msg/PolygonStamped` | CDR |
| `geometry_msgs.Pose` | `geometry_msgs/msg/Pose` | CDR |
| `geometry_msgs.Pose2D` | `geometry_msgs/msg/Pose2D` | CDR |
| `geometry_msgs.PoseArray` | `geometry_msgs/msg/PoseArray` | CDR |
| `geometry_msgs.PoseStamped` | `geometry_msgs/msg/PoseStamped` | CDR |
| `geometry_msgs.PoseWithCovariance` | `geometry_msgs/msg/PoseWithCovariance` | CDR |
| `geometry_msgs.PoseWithCovarianceStamped` | `geometry_msgs/msg/PoseWithCovarianceStamped` | CDR |
| `geometry_msgs.Quaternion` | `geometry_msgs/msg/Quaternion` | CDR |
| `geometry_msgs.QuaternionStamped` | `geometry_msgs/msg/QuaternionStamped` | CDR |
| `geometry_msgs.Transform` | `geometry_msgs/msg/Transform` | CDR |
| `geometry_msgs.TransformStamped` | `geometry_msgs/msg/TransformStamped` | CDR |
| `geometry_msgs.Twist` | `geometry_msgs/msg/Twist` | CDR |
| `geometry_msgs.TwistStamped` | `geometry_msgs/msg/TwistStamped` | CDR |
| `geometry_msgs.TwistWithCovariance` | `geometry_msgs/msg/TwistWithCovariance` | CDR |
| `geometry_msgs.TwistWithCovarianceStamped` | `geometry_msgs/msg/TwistWithCovarianceStamped` | CDR |
| `geometry_msgs.Vector3` | `geometry_msgs/msg/Vector3` | CDR |
| `geometry_msgs.Vector3Stamped` | `geometry_msgs/msg/Vector3Stamped` | CDR |
| `geometry_msgs.Wrench` | `geometry_msgs/msg/Wrench` | CDR |
| `geometry_msgs.WrenchStamped` | `geometry_msgs/msg/WrenchStamped` | CDR |
| `nav_msgs.GridCells` | `nav_msgs/msg/GridCells` | CDR |
| `nav_msgs.MapMetaData` | `nav_msgs/msg/MapMetaData` | CDR |
| `nav_msgs.OccupancyGrid` | `nav_msgs/msg/OccupancyGrid` | CDR |
| `nav_msgs.Odometry` | `nav_msgs/msg/Odometry` | CDR |
| `nav_msgs.Path` | `nav_msgs/msg/Path` | CDR |
| `sensor_msgs.BatteryState` | `sensor_msgs/msg/BatteryState` | CDR |
| `sensor_msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` | CDR |
| `sensor_msgs.ChannelFloat32` | `sensor_msgs/msg/ChannelFloat32` | CDR |
| `sensor_msgs.CompressedImage` | `sensor_msgs/msg/CompressedImage` | CDR |
| `sensor_msgs.FluidPressure` | `sensor_msgs/msg/FluidPressure` | CDR |
| `sensor_msgs.Illuminance` | `sensor_msgs/msg/Illuminance` | CDR |
| `sensor_msgs.Image` | `sensor_msgs/msg/Image` | CDR |
| `sensor_msgs.Imu` | `sensor_msgs/msg/Imu` | CDR |
| `sensor_msgs.JointState` | `sensor_msgs/msg/JointState` | CDR |
| `sensor_msgs.Joy` | `sensor_msgs/msg/Joy` | CDR |
| `sensor_msgs.JoyFeedback` | `sensor_msgs/msg/JoyFeedback` | CDR |
| `sensor_msgs.JoyFeedbackArray` | `sensor_msgs/msg/JoyFeedbackArray` | CDR |
| `sensor_msgs.LaserEcho` | `sensor_msgs/msg/LaserEcho` | CDR |
| `sensor_msgs.LaserScan` | `sensor_msgs/msg/LaserScan` | CDR |
| `sensor_msgs.MagneticField` | `sensor_msgs/msg/MagneticField` | CDR |
| `sensor_msgs.MultiDOFJointState` | `sensor_msgs/msg/MultiDOFJointState` | CDR |
| `sensor_msgs.MultiEchoLaserScan` | `sensor_msgs/msg/MultiEchoLaserScan` | CDR |
| `sensor_msgs.NavSatFix` | `sensor_msgs/msg/NavSatFix` | CDR |
| `sensor_msgs.NavSatStatus` | `sensor_msgs/msg/NavSatStatus` | CDR |
| `sensor_msgs.PointCloud` | `sensor_msgs/msg/PointCloud` | CDR |
| `sensor_msgs.PointCloud2` | `sensor_msgs/msg/PointCloud2` | CDR |
| `sensor_msgs.PointField` | `sensor_msgs/msg/PointField` | CDR |
| `sensor_msgs.Range` | `sensor_msgs/msg/Range` | CDR (zero: variance) |
| `sensor_msgs.RegionOfInterest` | `sensor_msgs/msg/RegionOfInterest` | CDR |
| `sensor_msgs.RelativeHumidity` | `sensor_msgs/msg/RelativeHumidity` | CDR |
| `sensor_msgs.Temperature` | `sensor_msgs/msg/Temperature` | CDR |
| `sensor_msgs.TimeReference` | `sensor_msgs/msg/TimeReference` | CDR |
| `shape_msgs.Mesh` | `shape_msgs/msg/Mesh` | CDR |
| `shape_msgs.MeshTriangle` | `shape_msgs/msg/MeshTriangle` | CDR |
| `shape_msgs.Plane` | `shape_msgs/msg/Plane` | CDR |
| `shape_msgs.SolidPrimitive` | `shape_msgs/msg/SolidPrimitive` | CDR (zero: polygon) |
| `std_msgs.Bool` | `std_msgs/msg/Bool` | CDR |
| `std_msgs.Byte` | `std_msgs/msg/Byte` | CDR |
| `std_msgs.ByteMultiArray` | `std_msgs/msg/ByteMultiArray` | CDR |
| `std_msgs.Char` | `std_msgs/msg/Char` | CDR |
| `std_msgs.ColorRGBA` | `std_msgs/msg/ColorRGBA` | CDR |
| `std_msgs.Duration` | `builtin_interfaces/msg/Duration` | CDR |
| `std_msgs.Empty` | `std_msgs/msg/Empty` | CDR |
| `std_msgs.Float32` | `std_msgs/msg/Float32` | CDR |
| `std_msgs.Float32MultiArray` | `std_msgs/msg/Float32MultiArray` | CDR |
| `std_msgs.Float64` | `std_msgs/msg/Float64` | CDR |
| `std_msgs.Float64MultiArray` | `std_msgs/msg/Float64MultiArray` | CDR |
| `std_msgs.Header` | `std_msgs/msg/Header` | CDR |
| `std_msgs.Int16` | `std_msgs/msg/Int16` | CDR |
| `std_msgs.Int16MultiArray` | `std_msgs/msg/Int16MultiArray` | CDR |
| `std_msgs.Int32` | `std_msgs/msg/Int32` | CDR |
| `std_msgs.Int32MultiArray` | `std_msgs/msg/Int32MultiArray` | CDR |
| `std_msgs.Int64` | `std_msgs/msg/Int64` | CDR |
| `std_msgs.Int64MultiArray` | `std_msgs/msg/Int64MultiArray` | CDR |
| `std_msgs.Int8` | `std_msgs/msg/Int8` | CDR |
| `std_msgs.Int8MultiArray` | `std_msgs/msg/Int8MultiArray` | CDR |
| `std_msgs.MultiArrayDimension` | `std_msgs/msg/MultiArrayDimension` | CDR |
| `std_msgs.MultiArrayLayout` | `std_msgs/msg/MultiArrayLayout` | CDR |
| `std_msgs.String` | `std_msgs/msg/String` | CDR |
| `std_msgs.Time` | `builtin_interfaces/msg/Time` | CDR |
| `std_msgs.UInt16` | `std_msgs/msg/UInt16` | CDR |
| `std_msgs.UInt16MultiArray` | `std_msgs/msg/UInt16MultiArray` | CDR |
| `std_msgs.UInt32` | `std_msgs/msg/UInt32` | CDR |
| `std_msgs.UInt32MultiArray` | `std_msgs/msg/UInt32MultiArray` | CDR |
| `std_msgs.UInt64` | `std_msgs/msg/UInt64` | CDR |
| `std_msgs.UInt64MultiArray` | `std_msgs/msg/UInt64MultiArray` | CDR |
| `std_msgs.UInt8` | `std_msgs/msg/UInt8` | CDR |
| `std_msgs.UInt8MultiArray` | `std_msgs/msg/UInt8MultiArray` | CDR |
| `stereo_msgs.DisparityImage` | `stereo_msgs/msg/DisparityImage` | CDR |
| `tf2_msgs.TF2Error` | `tf2_msgs/msg/TF2Error` | CDR |
| `tf2_msgs.TFMessage` | `tf2_msgs/msg/TFMessage` | CDR |
| `trajectory_msgs.JointTrajectory` | `trajectory_msgs/msg/JointTrajectory` | CDR |
| `trajectory_msgs.JointTrajectoryPoint` | `trajectory_msgs/msg/JointTrajectoryPoint` | CDR |
| `trajectory_msgs.MultiDOFJointTrajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | CDR |
| `trajectory_msgs.MultiDOFJointTrajectoryPoint` | `trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` | CDR |
| `vision_msgs.BoundingBox2D` | `vision_msgs/msg/BoundingBox2D` | CDR |
| `vision_msgs.BoundingBox2DArray` | `vision_msgs/msg/BoundingBox2DArray` | CDR |
| `vision_msgs.BoundingBox3D` | `vision_msgs/msg/BoundingBox3D` | CDR |
| `vision_msgs.BoundingBox3DArray` | `vision_msgs/msg/BoundingBox3DArray` | CDR |
| `vision_msgs.Classification` | `vision_msgs/msg/Classification` | CDR |
| `vision_msgs.Detection2D` | `vision_msgs/msg/Detection2D` | CDR |
| `vision_msgs.Detection2DArray` | `vision_msgs/msg/Detection2DArray` | CDR |
| `vision_msgs.Detection3D` | `vision_msgs/msg/Detection3D` | CDR |
| `vision_msgs.Detection3DArray` | `vision_msgs/msg/Detection3DArray` | CDR |
| `vision_msgs.LabelInfo` | `vision_msgs/msg/LabelInfo` | CDR |
| `vision_msgs.ObjectHypothesis` | `vision_msgs/msg/ObjectHypothesis` | CDR |
| `vision_msgs.ObjectHypothesisWithPose` | `vision_msgs/msg/ObjectHypothesisWithPose` | CDR |
| `vision_msgs.Point2D` | `vision_msgs/msg/Point2D` | CDR |
| `vision_msgs.Pose2D` | `vision_msgs/msg/Pose2D` | CDR |
| `vision_msgs.VisionClass` | `vision_msgs/msg/VisionClass` | CDR |
| `vision_msgs.VisionInfo` | `vision_msgs/msg/VisionInfo` | CDR |
| `visualization_msgs.ImageMarker` | `visualization_msgs/msg/ImageMarker` | CDR |
| `visualization_msgs.InteractiveMarker` | `visualization_msgs/msg/InteractiveMarker` | CDR |
| `visualization_msgs.InteractiveMarkerControl` | `visualization_msgs/msg/InteractiveMarkerControl` | CDR |
| `visualization_msgs.InteractiveMarkerFeedback` | `visualization_msgs/msg/InteractiveMarkerFeedback` | CDR |
| `visualization_msgs.InteractiveMarkerInit` | `visualization_msgs/msg/InteractiveMarkerInit` | CDR |
| `visualization_msgs.InteractiveMarkerPose` | `visualization_msgs/msg/InteractiveMarkerPose` | CDR |
| `visualization_msgs.InteractiveMarkerUpdate` | `visualization_msgs/msg/InteractiveMarkerUpdate` | CDR |
| `visualization_msgs.Marker` | `visualization_msgs/msg/Marker` | CDR (zero: texture_resource, texture, uv_coordinates, mesh_file) |
| `visualization_msgs.MarkerArray` | `visualization_msgs/msg/MarkerArray` | CDR |
| `visualization_msgs.MenuEntry` | `visualization_msgs/msg/MenuEntry` | CDR |

</details>

## Compatibility

| reader | `.db` | dimos MCAP | ROS 2 MCAP | how it's verified |
|---|---|---|---|---|
| dimos `SqliteStore` (main, desktop-gateway-v1.19.3) | ✔ | — | — | `tests/reference/check_dimos.py`: every stream decodes with the right payload class, count and source times. The registry row is byte-identical to the one Python writes |
| dimos `McapStore` (main, desktop-gateway-v1.19.3) | — | ✔ (codecs passed in, as with cc's recorder) | — | same script |
| Foxglove (`@foxglove/rosmsg` + `@foxglove/rosmsg2-serialization`, its decoders) | — | — | ✔ | `tests/reference/check_foxglove.ts`: all 170 mapped types, field by field |
| rosbags (Python, ROS 2) | — | — | ✔ | `tests/reference/check_rosbags.py`: the same messages |
| zenoh | 1.6.2, 1.10.1 | | | `compat/zenoh-1-6`, `compat/zenoh-1-10` loopback tests |

dimos readers older than the `memory2 → memory` rename (2026-08-14, dimos #3413 `chore/rename-memory2-to-memory`) are not supported: the registry names `dimos.memory.*` classes, as the Python store writes them today.

## Design notes

These follow cc's stated intentions where the requirements allow, and the deviations are listed.

- **Storage layout and time rules come from cc's recorder unchanged.** The table schema, the TF rows without a reception index, codec ids, MCAP channel metadata, `log_time` = reception and `publish_time` = source all come from cc's code: dimos #3615 `cc/rust-recorder` ("add experimental native SQLite and MCAP recorder"), #3924 `cc/feat/rust-recorder-cli` ("add experimental Rust --record", commit 8b880d77e1), #3970 `cc/fix/rust-recorder-chore` and #4392 `cc/feat/native-json-recording`. cc said it was "compatible with current python reader" ([Discord, 2026-09-02](https://discord.com/channels/1341146487186391173/1499980885636939956/1544809486538248302)). The `_streams` registry row, which his Python wrapper wrote before launching the binary, is written here byte for byte.
- **Unrepresentable times are rejected, never clamped.** This is cc's rule in `store/mcap.rs` ("Recording fails on an unrepresentable source or reception time instead of clamping it") and in #4370 `cc/test/cdr-record-replay`, where nanoseconds ≥ 1e9 and negative times are rejected. SQLite keeps negative JSON source times, as his recorder does. One deviation: a bad time rejects or falls back **that message** and counts it, instead of failing the whole recording, because a live controller shouldn't lose a session to one bad stamp.
- **CDR conventions** follow cc's CDR stack (#4246 `cc/feat/better-mcap-recorder`, #4367 `cc/feat/cdr-mcap-viewers`, #4445 `cc/feat/cdr-recording-converter`):
  - profile `ros2`, `message_encoding: cdr` (XCDR1, little-endian), `schema_encoding: ros2msg`;
  - the schema is named `package/msg/Type`; its text is the pinned Jazzy `.msg` with every dependency appended after an 80-`=` line and `MSG: package/Type`. He called that spelling load-bearing for Foxglove and Rerun;
  - `Time.nsec → nanosec`; length fields, fingerprints and `Header.seq` are dropped;
  - **TF is split into single-transform TFMessages**, each at its own stamp (#4246, #4370);
  - **images are raw `sensor_msgs/Image` by default**, losslessly including depth; JPEG only on request as `CompressedImage` with `format: jpeg`, keeping the original JPEG bytes (#4445);
  - the `dimos.*` channel metadata keys are kept on CDR channels too.
- **Upstream reference decoders, as cc asked.** On #4247 `cc/feat/remove-lcm-encoding` he wrote "we should use existing library but this is clearly manual impl?". There is no Rust crate that does dynamic, definition-driven CDR. So the encoder here is small and generic, and its output is checked against the real decoders (Foxglove's and rosbags) for every mapped type, the same check his `viewer-checker/check.mjs` does.
- **Where this deliberately differs from cc's direction:**
  - His final design (the 8-PR CDR stack, 2026-10) moves CDR onto the wire, so nothing transcodes; he closed the transcoding PR #4246 "in lieu of cdr refactor". This crate transcodes because today's dimos (and the Controller) still send LCM. Once CDR is on the wire, CDR payloads could pass straight through.
  - He ruled out an opaque fallback ("Unknown inputs fail preflight; there is no … opaque byte fallback", #4246). This crate's requirement is to never drop data, so types with no lossless ROS 2 mapping are kept as raw LCM channels in the `ros2` file. Set `strict_cdr` to get his behaviour.
  - Topics: his stack uses the stream name as-is (`odom`). Here the ROS 2 topic is whatever the caller names the stream. The Controller uses `/odom`, the ROS 2 convention, which keeps existing Foxglove layouts working.
- **Bounded memory, nothing silent.** Admission is bounded by message count and bytes. Over the bound, `record` returns false and counts a drop. Everything else is counted per stream, with the last error kept:
  - skipped by the caller,
  - rejected as invalid,
  - kept as raw-LCM fallback.

  Encoding runs on a thread pool, and one writer restores arrival order (cc's design).

## Development

```sh
cargo test                                                       # unit + integration (incl. a real kill-and-recover)
cargo run --example reference_fixture -- fixtures                # recordings + what readers should find
uv run tests/reference/check_rosbags.py fixtures/ros2.mcap
deno run -A tests/reference/check_foxglove.ts fixtures/ros2.mcap
(cd /path/to/dimos && uv run python /path/to/rust-recorder/tests/reference/check_dimos.py /path/to/fixtures --mcap)
(cd compat/zenoh-1-6 && cargo test); (cd compat/zenoh-1-10 && cargo test)
```

Licensed Apache-2.0. See NOTICE for the derivation from Dimensional Inc.'s recorder and the sources of the vendored definitions.
