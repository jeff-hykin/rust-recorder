//! ROS 2: the pinned Jazzy `.msg` definitions (msgs/ros2), the full schema text MCAP embeds, the field-by-field
//! mapping from a dimos-lcm type onto its ROS 2 counterpart, and CDR (XCDR1, little-endian) serialization.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, OnceLock};

use anyhow::{anyhow, bail, ensure, Result};

use crate::lcm::{self, Dim, MemberType, Prim, Value};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RosPrim {
    Bool,
    Byte,
    Char,
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
    Float32,
    Float64,
}

impl RosPrim {
    fn parse(name: &str) -> Option<RosPrim> {
        Some(match name {
            "bool" => RosPrim::Bool,
            "byte" => RosPrim::Byte,
            "char" => RosPrim::Char,
            "int8" => RosPrim::Int8,
            "uint8" => RosPrim::Uint8,
            "int16" => RosPrim::Int16,
            "uint16" => RosPrim::Uint16,
            "int32" => RosPrim::Int32,
            "uint32" => RosPrim::Uint32,
            "int64" => RosPrim::Int64,
            "uint64" => RosPrim::Uint64,
            "float32" => RosPrim::Float32,
            "float64" => RosPrim::Float64,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            RosPrim::Bool | RosPrim::Byte | RosPrim::Char | RosPrim::Int8 | RosPrim::Uint8 => 1,
            RosPrim::Int16 | RosPrim::Uint16 => 2,
            RosPrim::Int32 | RosPrim::Uint32 | RosPrim::Float32 => 4,
            RosPrim::Int64 | RosPrim::Uint64 | RosPrim::Float64 => 8,
        }
    }

    fn is_float(self) -> bool {
        matches!(self, RosPrim::Float32 | RosPrim::Float64)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RosBase {
    Prim(RosPrim),
    String,
    WString,
    /// `package/msg/Name`
    Msg(String),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RosArray {
    Scalar,
    Fixed(usize),
    /// unbounded or bounded: both are a length-prefixed sequence in CDR
    Sequence,
}

#[derive(Clone, Debug)]
pub struct RosField {
    pub name: String,
    pub base: RosBase,
    pub array: RosArray,
}

#[derive(Clone, Debug)]
pub struct RosMsg {
    /// `package/msg/Name`
    pub name: String,
    pub text: &'static str,
    pub fields: Vec<RosField>,
}

pub struct RosRegistry {
    pub messages: HashMap<String, RosMsg>,
    definitions: Mutex<HashMap<String, Arc<str>>>,
}

pub fn registry() -> &'static RosRegistry {
    static REGISTRY: OnceLock<RosRegistry> = OnceLock::new();
    REGISTRY.get_or_init(|| {
        let mut messages = HashMap::new();
        for (package, name, text) in crate::definitions::ROS2_DEFINITIONS {
            let parsed = parse_msg(package, name, text).unwrap_or_else(|error| panic!("msgs/ros2/{package}/{name}.msg: {error:#}"));
            messages.insert(parsed.name.clone(), parsed);
        }
        RosRegistry { messages, definitions: Mutex::default() }
    })
}

impl RosRegistry {
    /// The schema text for an MCAP `ros2msg` schema: the type's own .msg, then each type it uses (transitively, once)
    /// after a line of 80 `=` and `MSG: package/Name` (the rosbag2 / Foxglove layout).
    pub fn definition(&self, name: &str) -> Result<Arc<str>> {
        if let Some(text) = self.definitions.lock().unwrap().get(name) {
            return Ok(text.clone());
        }
        let root = self.messages.get(name).ok_or_else(|| anyhow!("no ROS 2 definition for {name}"))?;
        let mut text = root.text.trim_end().to_string();
        text.push('\n');
        let mut seen = HashSet::from([name.to_string()]);
        let mut pending: Vec<&RosMsg> = vec![root];
        let mut order = Vec::new();
        while let Some(message) = pending.pop() {
            let mut children = Vec::new();
            for field in &message.fields {
                if let RosBase::Msg(child) = &field.base {
                    if seen.insert(child.clone()) {
                        let child_message = self.messages.get(child).ok_or_else(|| anyhow!("{name} uses {child}, which has no definition"))?;
                        order.push(child_message);
                        children.push(child_message);
                    }
                }
            }
            // depth-first in field order
            pending.extend(children.into_iter().rev());
        }
        for message in order {
            text.push_str(&"=".repeat(80));
            text.push_str("\nMSG: ");
            text.push_str(&message.name.replace("/msg/", "/"));
            text.push('\n');
            text.push_str(message.text.trim_end());
            text.push('\n');
        }
        let text: Arc<str> = text.into();
        self.definitions.lock().unwrap().insert(name.to_string(), text.clone());
        Ok(text)
    }
}

fn parse_msg(package: &str, name: &str, text: &'static str) -> Result<RosMsg> {
    let mut fields = Vec::new();
    for raw_line in text.lines() {
        let line = strip_comment(raw_line).trim();
        if line.is_empty() {
            continue;
        }
        let (type_token, rest) = line.split_once(char::is_whitespace).ok_or_else(|| anyhow!("malformed line {line:?}"))?;
        let rest = rest.trim_start();
        let field_name: String = rest.chars().take_while(|character| character.is_alphanumeric() || *character == '_').collect();
        ensure!(!field_name.is_empty(), "malformed line {line:?}");
        if rest[field_name.len()..].trim_start().starts_with('=') {
            continue; // a constant
        }
        let (base_token, array) = match type_token.find('[') {
            None => (type_token, RosArray::Scalar),
            Some(open) => {
                let size = type_token[open + 1..].trim_end_matches(']');
                let array = if size.is_empty() || size.starts_with("<=") {
                    RosArray::Sequence
                } else {
                    RosArray::Fixed(size.parse().map_err(|_| anyhow!("bad array size in {type_token}"))?)
                };
                (&type_token[..open], array)
            }
        };
        let base = if base_token == "string" || base_token.starts_with("string<=") {
            RosBase::String
        } else if base_token == "wstring" || base_token.starts_with("wstring<=") {
            RosBase::WString
        } else if let Some(prim) = RosPrim::parse(base_token) {
            RosBase::Prim(prim)
        } else if let Some((other_package, other_name)) = base_token.split_once('/') {
            RosBase::Msg(format!("{other_package}/msg/{}", other_name.trim_start_matches("msg/")))
        } else if base_token == "Header" {
            RosBase::Msg("std_msgs/msg/Header".into())
        } else {
            RosBase::Msg(format!("{package}/msg/{base_token}"))
        };
        fields.push(RosField { name: field_name, base, array });
    }
    Ok(RosMsg { name: format!("{package}/msg/{name}"), text, fields })
}

fn strip_comment(line: &str) -> &str {
    let mut in_quote: Option<char> = None;
    for (index, character) in line.char_indices() {
        match (character, in_quote) {
            ('"' | '\'', None) => in_quote = Some(character),
            (quote, Some(open)) if quote == open => in_quote = None,
            ('#', None) => return &line[..index],
            _ => {}
        }
    }
    line
}

/// The ROS 2 type a dimos-lcm type corresponds to: `pkg.Name` → `pkg/msg/Name`, except dimos-lcm's ROS 1-style
/// std_msgs Time and Duration, which ROS 2 moved to builtin_interfaces.
pub fn ros_name_for(lcm_name: &str) -> String {
    match lcm_name {
        "std_msgs.Time" => "builtin_interfaces/msg/Time".into(),
        "std_msgs.Duration" => "builtin_interfaces/msg/Duration".into(),
        _ => {
            let (package, name) = lcm_name.split_once('.').unwrap_or(("", lcm_name));
            format!("{package}/msg/{name}")
        }
    }
}

/// LCM members a ROS 2 type deliberately has no place for.
fn droppable(lcm_type: &str, member: &str) -> bool {
    // ROS 2 removed Header.seq
    lcm_type == "std_msgs.Header" && member == "seq"
}

/// How one ROS 2 field is filled from the LCM value.
#[derive(Debug)]
enum Fill {
    /// from LCM member `index`
    Member { index: usize, kind: Kind },
    /// the ROS 2 field has no LCM counterpart: written as its zero value
    Default(Kind),
}

#[derive(Debug)]
enum Kind {
    Prim { from: Prim, to: RosPrim, array: RosArray },
    String { array: RosArray },
    Struct { map: Arc<StructMap>, array: RosArray },
}

/// A lossless mapping of an LCM struct onto a ROS 2 message.
#[derive(Debug)]
pub struct StructMap {
    pub lcm: String,
    pub ros: String,
    fills: Vec<(String, Fill)>,
    /// builtin_interfaces Time/Duration: nanosec must be in [0, 1e9)
    is_time: bool,
    /// ROS 2 fields written as zero because LCM has no such member (reported, never silent)
    pub defaulted: Vec<String>,
}

/// Why a type is or isn't recorded as CDR.
#[derive(Clone, Debug)]
pub enum Mapping {
    Cdr(Arc<StructMap>),
    Unmapped(String),
}

pub fn mapping(lcm_name: &str) -> Mapping {
    static CACHE: OnceLock<Mutex<HashMap<String, Mapping>>> = OnceLock::new();
    let cache = CACHE.get_or_init(Mutex::default);
    if let Some(found) = cache.lock().unwrap().get(lcm_name) {
        return found.clone();
    }
    let result = match build_map(lcm_name, &ros_name_for(lcm_name), &mut Vec::new()) {
        Ok(map) => Mapping::Cdr(map),
        Err(error) => Mapping::Unmapped(format!("{error:#}")),
    };
    cache.lock().unwrap().insert(lcm_name.to_string(), result.clone());
    result
}

fn build_map(lcm_name: &str, ros_name: &str, stack: &mut Vec<String>) -> Result<Arc<StructMap>> {
    ensure!(!stack.iter().any(|seen| seen == lcm_name), "{lcm_name} is recursive");
    let lcm_struct = lcm::registry().get(lcm_name).ok_or_else(|| anyhow!("no LCM definition for {lcm_name}"))?;
    let ros = registry().messages.get(ros_name).ok_or_else(|| anyhow!("no ROS 2 counterpart ({ros_name})"))?;
    stack.push(lcm_name.to_string());
    let mut consumed = vec![false; lcm_struct.members.len()];
    for member in &lcm_struct.members {
        for dim in &member.dims {
            if let Dim::Var(length) = dim {
                if let Some(index) = lcm_struct.member_index(length) {
                    consumed[index] = true;
                }
            }
        }
    }
    let mut fills = Vec::new();
    let mut defaulted = Vec::new();
    for field in &ros.fields {
        let found = lcm_struct
            .member_index(&field.name)
            .or_else(|| lcm_struct.members.iter().position(|member| member.name.eq_ignore_ascii_case(&field.name)))
            .or_else(|| (field.name == "nanosec").then(|| lcm_struct.member_index("nsec")).flatten());
        let fill = match found {
            Some(index) => {
                let member = &lcm_struct.members[index];
                ensure!(!consumed[index], "{lcm_name}.{} is used twice", member.name);
                consumed[index] = true;
                let kind = match_kind(lcm_name, member, field, stack)?;
                Fill::Member { index, kind }
            }
            None => {
                defaulted.push(field.name.clone());
                Fill::Default(default_kind(field, stack)?)
            }
        };
        fills.push((field.name.clone(), fill));
    }
    for (member, used) in lcm_struct.members.iter().zip(&consumed) {
        ensure!(*used || droppable(lcm_name, &member.name), "{lcm_name}.{} has no field in {ros_name}", member.name);
    }
    stack.pop();
    Ok(Arc::new(StructMap {
        lcm: lcm_name.to_string(),
        ros: ros_name.to_string(),
        fills,
        is_time: matches!(ros_name, "builtin_interfaces/msg/Time" | "builtin_interfaces/msg/Duration"),
        defaulted,
    }))
}

fn match_kind(lcm_name: &str, member: &lcm::Member, field: &RosField, stack: &mut Vec<String>) -> Result<Kind> {
    let context = || format!("{lcm_name}.{} → {}", member.name, field.name);
    ensure!(member.dims.len() <= 1, "{}: multi-dimensional arrays are not mapped", context());
    let array = match (member.dims.first(), field.array) {
        (None, RosArray::Scalar) => RosArray::Scalar,
        (Some(Dim::Fixed(lcm_size)), RosArray::Fixed(ros_size)) if *lcm_size == ros_size => RosArray::Fixed(ros_size),
        (Some(_), RosArray::Sequence) => RosArray::Sequence,
        (lcm_dim, ros_array) => bail!("{}: LCM shape {lcm_dim:?} does not fit ROS 2 shape {ros_array:?}", context()),
    };
    Ok(match (&member.ty, &field.base) {
        (MemberType::Prim(Prim::String), RosBase::String) => Kind::String { array },
        (MemberType::Prim(from), RosBase::Prim(to)) => {
            ensure!(lossless(*from, *to), "{}: {from:?} does not fit {to:?}", context());
            Kind::Prim { from: *from, to: *to, array }
        }
        (MemberType::Struct(lcm_child), RosBase::Msg(ros_child)) => Kind::Struct { map: build_map(lcm_child, ros_child, stack)?, array },
        (lcm_type, ros_type) => bail!("{}: {lcm_type:?} does not fit {ros_type:?}", context()),
    })
}

fn default_kind(field: &RosField, stack: &mut Vec<String>) -> Result<Kind> {
    Ok(match &field.base {
        RosBase::Prim(to) => Kind::Prim { from: Prim::Int64, to: *to, array: field.array },
        RosBase::String => Kind::String { array: field.array },
        RosBase::WString => bail!("{}: wstring is not supported", field.name),
        RosBase::Msg(name) => Kind::Struct { map: default_map(name, stack)?, array: field.array },
    })
}

/// A ROS 2 message written entirely as zeros (a field LCM lacks).
fn default_map(ros_name: &str, stack: &mut Vec<String>) -> Result<Arc<StructMap>> {
    let ros = registry().messages.get(ros_name).ok_or_else(|| anyhow!("no ROS 2 definition for {ros_name}"))?;
    let mut fills = Vec::new();
    for field in &ros.fields {
        fills.push((field.name.clone(), Fill::Default(default_kind(field, stack)?)));
    }
    Ok(Arc::new(StructMap { lcm: String::new(), ros: ros_name.to_string(), fills, is_time: false, defaulted: vec![] }))
}

/// Whether every value of the LCM primitive survives as the ROS 2 one. dimos-lcm stores ROS unsigned types in the
/// same-width signed LCM type, so those reinterpret bit for bit.
fn lossless(from: Prim, to: RosPrim) -> bool {
    use RosPrim::*;
    match from {
        Prim::Boolean => matches!(to, Bool | Byte | Char | Int8 | Uint8),
        Prim::Byte | Prim::Int8 => matches!(to, Byte | Char | Int8 | Uint8) || (!to.is_float() && to.size() > 1 && to != Bool),
        Prim::Int16 => matches!(to, Int16 | Uint16 | Int32 | Int64 | Float32 | Float64),
        Prim::Int32 => matches!(to, Int32 | Uint32 | Int64 | Float64),
        Prim::Int64 => matches!(to, Int64 | Uint64),
        Prim::Float => matches!(to, Float32 | Float64),
        Prim::Double => to == Float64,
        Prim::String => false,
    }
}

/// CDR (XCDR1, little-endian) for an LCM value, through a mapping.
pub fn encode_cdr(map: &StructMap, value: &Value) -> Result<Vec<u8>> {
    let mut writer = CdrWriter { out: vec![0x00, 0x01, 0x00, 0x00] };
    writer.write_struct(map, Some(value))?;
    Ok(writer.out)
}

struct CdrWriter {
    out: Vec<u8>,
}

impl CdrWriter {
    /// Alignment is relative to the byte after the 4-byte encapsulation header.
    fn align(&mut self, size: usize) {
        let position = self.out.len() - 4;
        let padding = (size - position % size) % size;
        self.out.resize(self.out.len() + padding, 0);
    }

    fn write_u32(&mut self, value: u32) {
        self.align(4);
        self.out.extend_from_slice(&value.to_le_bytes());
    }

    fn write_struct(&mut self, map: &StructMap, value: Option<&Value>) -> Result<()> {
        let fields = match value {
            Some(value) => Some(value.fields().ok_or_else(|| anyhow!("{}: expected a struct", map.lcm))?),
            None => None,
        };
        if map.is_time {
            if let Some(fields) = fields {
                let nanos = map
                    .fills
                    .iter()
                    .find(|(name, _)| name == "nanosec")
                    .and_then(|(_, fill)| match fill {
                        Fill::Member { index, .. } => fields[*index].as_i64(),
                        Fill::Default(_) => Some(0),
                    })
                    .unwrap_or(0);
                ensure!((0..1_000_000_000).contains(&nanos), "{}: nanoseconds {nanos} outside [0, 1e9)", map.ros);
            }
        }
        if map.fills.is_empty() {
            // rosidl gives an empty message one uint8 (structure_needs_at_least_one_member)
            self.out.push(0);
        }
        for (name, fill) in &map.fills {
            let result = match (fill, fields) {
                (Fill::Member { index, kind }, Some(fields)) => self.write_kind(kind, Some(&fields[*index])),
                (Fill::Member { kind, .. }, None) | (Fill::Default(kind), _) => self.write_kind(kind, None),
            };
            result.map_err(|error| anyhow!("{}.{name}: {error:#}", map.ros))?;
        }
        Ok(())
    }

    fn write_kind(&mut self, kind: &Kind, value: Option<&Value>) -> Result<()> {
        let array = match kind {
            Kind::Prim { array, .. } | Kind::String { array } | Kind::Struct { array, .. } => *array,
        };
        if array == RosArray::Scalar {
            return self.write_element(kind, value);
        }
        let fixed = match array {
            RosArray::Fixed(size) => Some(size),
            _ => None,
        };
        // packed primitive arrays go through in one pass
        if let (Kind::Prim { from, to, .. }, Some(Value::Bytes(bytes) | Value::Prims(_, bytes))) = (kind, value) {
            let count = bytes.len() / from.size();
            if let Some(size) = fixed {
                ensure!(count == size, "{count} elements for a fixed array of {size}");
            } else {
                self.write_u32(count as u32);
            }
            return self.write_packed(*from, *to, bytes);
        }
        let items: &[Value] = match value {
            Some(Value::Array(items)) => items,
            None => &[],
            Some(other) => bail!("expected an array, got {other:?}"),
        };
        match fixed {
            Some(size) => {
                ensure!(value.is_none() || items.len() == size, "{} elements for a fixed array of {size}", items.len());
                for index in 0..size {
                    self.write_element(kind, items.get(index))?;
                }
            }
            None => {
                self.write_u32(items.len() as u32);
                for item in items {
                    self.write_element(kind, Some(item))?;
                }
            }
        }
        Ok(())
    }

    fn write_packed(&mut self, from: Prim, to: RosPrim, bytes: &[u8]) -> Result<()> {
        if from.size() == 1 && to.size() == 1 {
            self.out.extend_from_slice(bytes);
            return Ok(());
        }
        self.align(to.size());
        if from.size() == to.size() && from.is_float() == to.is_float() {
            // same width: big-endian → little-endian
            self.out.reserve(bytes.len());
            for chunk in bytes.chunks_exact(from.size()) {
                self.out.extend(chunk.iter().rev());
            }
            return Ok(());
        }
        for chunk in bytes.chunks_exact(from.size()) {
            let value = decode_be(from, chunk);
            self.write_prim_value(to, &value)?;
        }
        Ok(())
    }

    fn write_element(&mut self, kind: &Kind, value: Option<&Value>) -> Result<()> {
        match kind {
            Kind::Prim { to, .. } => self.write_prim_value(*to, value.unwrap_or(&Value::Int(0))),
            Kind::String { .. } => {
                let bytes: &[u8] = match value {
                    Some(Value::Str(bytes)) => bytes,
                    None => &[],
                    Some(other) => bail!("expected a string, got {other:?}"),
                };
                ensure!(!bytes.contains(&0), "string contains a NUL byte");
                self.write_u32(bytes.len() as u32 + 1);
                self.out.extend_from_slice(bytes);
                self.out.push(0);
                Ok(())
            }
            Kind::Struct { map, .. } => self.write_struct(map, value),
        }
    }

    fn write_prim_value(&mut self, to: RosPrim, value: &Value) -> Result<()> {
        self.align(to.size());
        let integer = || value.as_i64().ok_or_else(|| anyhow!("expected an integer, got {value:?}"));
        match to {
            RosPrim::Bool => self.out.push((integer()? != 0) as u8),
            RosPrim::Byte | RosPrim::Char | RosPrim::Uint8 | RosPrim::Int8 => self.out.push(integer()? as u8),
            RosPrim::Int16 | RosPrim::Uint16 => self.out.extend_from_slice(&(integer()? as u16).to_le_bytes()),
            RosPrim::Int32 | RosPrim::Uint32 => self.out.extend_from_slice(&(integer()? as u32).to_le_bytes()),
            RosPrim::Int64 | RosPrim::Uint64 => self.out.extend_from_slice(&(integer()? as u64).to_le_bytes()),
            RosPrim::Float32 => {
                let number = value.as_f64().ok_or_else(|| anyhow!("expected a number, got {value:?}"))?;
                self.out.extend_from_slice(&(number as f32).to_le_bytes())
            }
            RosPrim::Float64 => {
                let number = value.as_f64().ok_or_else(|| anyhow!("expected a number, got {value:?}"))?;
                self.out.extend_from_slice(&number.to_le_bytes())
            }
        }
        Ok(())
    }
}

fn decode_be(prim: Prim, bytes: &[u8]) -> Value {
    match prim {
        Prim::Int16 => Value::Int(i16::from_be_bytes(bytes.try_into().unwrap()) as i64),
        Prim::Int32 => Value::Int(i32::from_be_bytes(bytes.try_into().unwrap()) as i64),
        Prim::Int64 => Value::Int(i64::from_be_bytes(bytes.try_into().unwrap())),
        Prim::Float => Value::Float(f32::from_be_bytes(bytes.try_into().unwrap()) as f64),
        Prim::Double => Value::Float(f64::from_be_bytes(bytes.try_into().unwrap())),
        Prim::Byte | Prim::Boolean => Value::Int(bytes[0] as i64),
        Prim::Int8 => Value::Int(bytes[0] as i8 as i64),
        Prim::String => Value::Int(0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn definitions_append_dependencies_once_in_the_rosbag2_layout() {
        let text = registry().definition("geometry_msgs/msg/PoseStamped").unwrap();
        assert!(text.starts_with("# A Pose with reference coordinate frame and timestamp"));
        for dependency in ["std_msgs/Header", "builtin_interfaces/Time", "geometry_msgs/Pose", "geometry_msgs/Point", "geometry_msgs/Quaternion"] {
            assert_eq!(text.matches(&format!("\nMSG: {dependency}\n")).count(), 1, "{dependency}");
        }
        assert!(text.contains(&format!("\n{}\nMSG: std_msgs/Header\n", "=".repeat(80))));
    }

    #[test]
    fn every_ros_definition_resolves_its_dependencies() {
        for name in registry().messages.keys() {
            registry().definition(name).unwrap_or_else(|error| panic!("{name}: {error:#}"));
        }
    }

    #[test]
    fn header_drops_seq_and_time_maps_nsec_to_nanosec() {
        let Mapping::Cdr(map) = mapping("std_msgs.Header") else { panic!() };
        assert_eq!(map.ros, "std_msgs/msg/Header");
        assert!(map.defaulted.is_empty());
        let header = Value::Struct(vec![Value::Int(9), Value::Struct(vec![Value::Int(12), Value::Int(34)]), Value::Str(b"map".to_vec())]);
        let cdr = encode_cdr(&map, &header).unwrap();
        assert_eq!(cdr, [0, 1, 0, 0, 12, 0, 0, 0, 34, 0, 0, 0, 4, 0, 0, 0, b'm', b'a', b'p', 0]);
        let bad = Value::Struct(vec![Value::Int(9), Value::Struct(vec![Value::Int(12), Value::Int(1_000_000_000)]), Value::Str(vec![])]);
        assert!(encode_cdr(&map, &bad).is_err());
    }

    #[test]
    fn camera_info_matches_the_ros2_lowercase_names() {
        let Mapping::Cdr(map) = mapping("sensor_msgs.CameraInfo") else { panic!("{:?}", mapping("sensor_msgs.CameraInfo")) };
        assert!(map.defaulted.is_empty());
    }
}

/// The ROS 2 message a mapping produces from an LCM value, as JSON (field names and number types as ROS 2 sees them;
/// byte arrays as number lists). What a CDR decoder should read back.
pub fn to_ros_json(map: &StructMap, value: Option<&Value>) -> Result<serde_json::Value> {
    let fields = value.map(|value| value.fields().ok_or_else(|| anyhow!("{}: expected a struct", map.lcm))).transpose()?;
    let mut out = serde_json::Map::new();
    for (name, fill) in &map.fills {
        let (kind, field) = match (fill, fields) {
            (Fill::Member { index, kind }, Some(fields)) => (kind, Some(&fields[*index])),
            (Fill::Member { kind, .. }, None) | (Fill::Default(kind), _) => (kind, None),
        };
        out.insert(name.clone(), kind_json(kind, field)?);
    }
    Ok(serde_json::Value::Object(out))
}

fn kind_json(kind: &Kind, value: Option<&Value>) -> Result<serde_json::Value> {
    let array = match kind {
        Kind::Prim { array, .. } | Kind::String { array } | Kind::Struct { array, .. } => *array,
    };
    if array == RosArray::Scalar {
        return element_json(kind, value);
    }
    let items: Vec<Value> = match value {
        Some(Value::Bytes(bytes)) => {
            let Kind::Prim { from, .. } = kind else { bail!("bytes for a non-primitive") };
            bytes.iter().map(|byte| if *from == Prim::Int8 { Value::Int(*byte as i8 as i64) } else { Value::Int(*byte as i64) }).collect()
        }
        Some(Value::Prims(prim, bytes)) => bytes.chunks_exact(prim.size()).map(|chunk| decode_be(*prim, chunk)).collect(),
        Some(Value::Array(items)) => items.clone(),
        None => match array {
            RosArray::Fixed(size) => return (0..size).map(|_| element_json(kind, None)).collect::<Result<Vec<_>>>().map(serde_json::Value::Array),
            _ => vec![],
        },
        Some(other) => bail!("expected an array, got {other:?}"),
    };
    items.iter().map(|item| element_json(kind, Some(item))).collect::<Result<Vec<_>>>().map(serde_json::Value::Array)
}

fn element_json(kind: &Kind, value: Option<&Value>) -> Result<serde_json::Value> {
    Ok(match kind {
        Kind::Prim { to, .. } => {
            let value = value.cloned().unwrap_or(Value::Int(0));
            match to {
                RosPrim::Bool => serde_json::json!(value.as_i64().unwrap_or(0) != 0),
                RosPrim::Float32 => serde_json::json!(value.as_f64().unwrap_or(0.0) as f32 as f64),
                RosPrim::Float64 => serde_json::json!(value.as_f64().unwrap_or(0.0)),
                RosPrim::Int8 => serde_json::json!(value.as_i64().unwrap_or(0) as i8),
                RosPrim::Byte | RosPrim::Char | RosPrim::Uint8 => serde_json::json!(value.as_i64().unwrap_or(0) as u8),
                RosPrim::Int16 => serde_json::json!(value.as_i64().unwrap_or(0) as i16),
                RosPrim::Uint16 => serde_json::json!(value.as_i64().unwrap_or(0) as u16),
                RosPrim::Int32 => serde_json::json!(value.as_i64().unwrap_or(0) as i32),
                RosPrim::Uint32 => serde_json::json!(value.as_i64().unwrap_or(0) as u32),
                RosPrim::Int64 => serde_json::json!(value.as_i64().unwrap_or(0)),
                RosPrim::Uint64 => serde_json::json!(value.as_i64().unwrap_or(0) as u64),
            }
        }
        Kind::String { .. } => match value {
            Some(Value::Str(bytes)) => serde_json::json!(String::from_utf8_lossy(bytes)),
            _ => serde_json::json!(""),
        },
        Kind::Struct { map, .. } => to_ros_json(map, value)?,
    })
}
