//! LCM, driven by the vendored dimos-lcm definitions (msgs/lcm): parsing `.lcm` files, the type fingerprint lcm-gen
//! computes, and a generic big-endian decode/encode into `Value`, so any dimos-lcm type is understood without codegen.

use std::collections::HashMap;
use std::sync::OnceLock;

use anyhow::{anyhow, bail, ensure, Context, Result};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Prim {
    Int8,
    Int16,
    Int32,
    Int64,
    Float,
    Double,
    Byte,
    Boolean,
    String,
}

impl Prim {
    fn parse(name: &str) -> Option<Prim> {
        Some(match name {
            "int8_t" => Prim::Int8,
            "int16_t" => Prim::Int16,
            "int32_t" => Prim::Int32,
            "int64_t" => Prim::Int64,
            "float" => Prim::Float,
            "double" => Prim::Double,
            "byte" => Prim::Byte,
            "boolean" => Prim::Boolean,
            "string" => Prim::String,
            _ => return None,
        })
    }

    /// The name lcm-gen hashes into the fingerprint.
    fn lcm_name(self) -> &'static str {
        match self {
            Prim::Int8 => "int8_t",
            Prim::Int16 => "int16_t",
            Prim::Int32 => "int32_t",
            Prim::Int64 => "int64_t",
            Prim::Float => "float",
            Prim::Double => "double",
            Prim::Byte => "byte",
            Prim::Boolean => "boolean",
            Prim::String => "string",
        }
    }

    /// Bytes on the wire (0 for string).
    pub fn size(self) -> usize {
        match self {
            Prim::Int8 | Prim::Byte | Prim::Boolean => 1,
            Prim::Int16 => 2,
            Prim::Int32 | Prim::Float => 4,
            Prim::Int64 | Prim::Double => 8,
            Prim::String => 0,
        }
    }

    pub fn is_float(self) -> bool {
        matches!(self, Prim::Float | Prim::Double)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MemberType {
    Prim(Prim),
    /// full name, `package.Name`
    Struct(String),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Dim {
    Fixed(usize),
    /// the earlier member holding the length
    Var(String),
}

#[derive(Clone, Debug)]
pub struct Member {
    pub name: String,
    pub ty: MemberType,
    pub dims: Vec<Dim>,
}

#[derive(Clone, Debug)]
pub struct Struct {
    /// `package.Name`
    pub name: String,
    pub members: Vec<Member>,
    base_hash: u64,
}

impl Struct {
    pub fn member_index(&self, name: &str) -> Option<usize> {
        self.members.iter().position(|member| member.name == name)
    }
}

/// A decoded LCM value. 1-D primitive arrays stay packed (`Bytes`, `Prims`) so large payloads are copied once.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Int(i64),
    Float(f64),
    Bool(bool),
    /// without the trailing NUL
    Str(Vec<u8>),
    /// a byte, int8_t or boolean array
    Bytes(Vec<u8>),
    /// a packed big-endian array of int16/int32/int64/float/double
    Prims(Prim, Vec<u8>),
    Array(Vec<Value>),
    Struct(Vec<Value>),
}

impl Value {
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Value::Int(value) => Some(*value),
            Value::Bool(value) => Some(*value as i64),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Float(value) => Some(*value),
            Value::Int(value) => Some(*value as f64),
            _ => None,
        }
    }

    pub fn fields(&self) -> Option<&[Value]> {
        match self {
            Value::Struct(fields) => Some(fields),
            _ => None,
        }
    }

    pub fn fields_mut(&mut self) -> Option<&mut Vec<Value>> {
        match self {
            Value::Struct(fields) => Some(fields),
            _ => None,
        }
    }

    /// Number of elements in an array value.
    pub fn len(&self, element: Prim) -> Option<usize> {
        match self {
            Value::Bytes(bytes) => Some(bytes.len()),
            Value::Prims(_, bytes) => Some(bytes.len() / element.size().max(1)),
            Value::Array(items) => Some(items.len()),
            _ => None,
        }
    }
}

/// Every struct in msgs/lcm, by `package.Name`.
pub struct Registry {
    pub structs: HashMap<String, Struct>,
    fingerprints: HashMap<String, u64>,
}

pub fn registry() -> &'static Registry {
    static REGISTRY: OnceLock<Registry> = OnceLock::new();
    REGISTRY.get_or_init(|| {
        let mut structs = HashMap::new();
        for (file, text) in crate::definitions::LCM_DEFINITIONS {
            for parsed in parse(text).unwrap_or_else(|error| panic!("msgs/lcm/{file}.lcm: {error:#}")) {
                structs.insert(parsed.name.clone(), parsed);
            }
        }
        let mut registry = Registry { structs, fingerprints: HashMap::new() };
        let names: Vec<String> = registry.structs.keys().cloned().collect();
        for name in names {
            if let Ok(fingerprint) = registry.compute_fingerprint(&name, &mut Vec::new()) {
                registry.fingerprints.insert(name, fingerprint);
            }
        }
        registry
    })
}

impl Registry {
    pub fn get(&self, name: &str) -> Option<&Struct> {
        self.structs.get(name)
    }

    /// The 8-byte hash an encoded message of this type starts with.
    pub fn fingerprint(&self, name: &str) -> Option<u64> {
        self.fingerprints.get(name).copied()
    }

    fn compute_fingerprint(&self, name: &str, parents: &mut Vec<String>) -> Result<u64> {
        if parents.iter().any(|parent| parent == name) {
            return Ok(0);
        }
        let definition = self.get(name).ok_or_else(|| anyhow!("unknown LCM type {name}"))?;
        parents.push(name.to_string());
        let mut hash = definition.base_hash;
        for member in &definition.members {
            if let MemberType::Struct(child) = &member.ty {
                hash = hash.wrapping_add(self.compute_fingerprint(child, parents)?);
            }
        }
        parents.pop();
        Ok(hash.rotate_left(1))
    }

    /// Decodes a whole top-level message, checking its fingerprint.
    pub fn decode(&self, name: &str, data: &[u8]) -> Result<Value> {
        self.decode_prefix(name, data, None)
    }

    /// Decodes a top-level message up to and including member `stop_after` (all of it when None).
    pub fn decode_prefix(&self, name: &str, data: &[u8], stop_after: Option<usize>) -> Result<Value> {
        let expected = self.fingerprint(name).ok_or_else(|| anyhow!("unknown LCM type {name}"))?;
        ensure!(data.len() >= 8, "{name}: {} bytes is too short for an LCM message", data.len());
        let actual = u64::from_be_bytes(data[..8].try_into().unwrap());
        ensure!(
            actual == expected,
            "payload is not an LCM {name} (fingerprint {actual:016x}, expected {expected:016x})"
        );
        let mut reader = Reader { data, offset: 8 };
        let value = self.decode_struct(name, &mut reader, stop_after)?;
        if stop_after.is_none() {
            ensure!(reader.offset == data.len(), "{name}: {} trailing bytes", data.len() - reader.offset);
        }
        Ok(value)
    }

    fn decode_struct(&self, name: &str, reader: &mut Reader, stop_after: Option<usize>) -> Result<Value> {
        let definition = self.get(name).ok_or_else(|| anyhow!("unknown LCM type {name}"))?;
        let mut fields: Vec<Value> = Vec::with_capacity(definition.members.len());
        for (index, member) in definition.members.iter().enumerate() {
            let lengths = member
                .dims
                .iter()
                .map(|dim| match dim {
                    Dim::Fixed(size) => Ok(*size),
                    Dim::Var(length) => {
                        let at = definition
                            .member_index(length)
                            .filter(|at| *at < index)
                            .ok_or_else(|| anyhow!("{name}.{}: length {length} is not an earlier member", member.name))?;
                        let count = fields[at].as_i64().ok_or_else(|| anyhow!("{name}.{length} is not an integer"))?;
                        usize::try_from(count).map_err(|_| anyhow!("{name}.{length} is negative ({count})"))
                    }
                })
                .collect::<Result<Vec<usize>>>()?;
            let value = self
                .decode_dims(&member.ty, &lengths, reader)
                .with_context(|| format!("{name}.{}", member.name))?;
            fields.push(value);
            if stop_after == Some(index) {
                break;
            }
        }
        Ok(Value::Struct(fields))
    }

    fn decode_dims(&self, ty: &MemberType, lengths: &[usize], reader: &mut Reader) -> Result<Value> {
        let Some((&count, rest)) = lengths.split_first() else {
            return self.decode_one(ty, reader);
        };
        if rest.is_empty() {
            if let MemberType::Prim(prim) = ty {
                match prim {
                    Prim::Byte | Prim::Int8 | Prim::Boolean => return Ok(Value::Bytes(reader.take(count)?.to_vec())),
                    Prim::String => {}
                    _ => {
                        let bytes = count.checked_mul(prim.size()).ok_or_else(|| anyhow!("array too large"))?;
                        return Ok(Value::Prims(*prim, reader.take(bytes)?.to_vec()));
                    }
                }
            }
        }
        // every element takes at least one byte, so a corrupt count fails here instead of allocating
        ensure!(count <= reader.remaining(), "array of {count} exceeds the {} bytes left", reader.remaining());
        let mut items = Vec::with_capacity(count);
        for _ in 0..count {
            items.push(self.decode_dims(ty, rest, reader)?);
        }
        Ok(Value::Array(items))
    }

    fn decode_one(&self, ty: &MemberType, reader: &mut Reader) -> Result<Value> {
        Ok(match ty {
            MemberType::Struct(name) => self.decode_struct(name, reader, None)?,
            MemberType::Prim(prim) => match prim {
                Prim::Int8 => Value::Int(reader.take(1)?[0] as i8 as i64),
                Prim::Byte => Value::Int(reader.take(1)?[0] as i64),
                Prim::Boolean => Value::Bool(reader.take(1)?[0] != 0),
                Prim::Int16 => Value::Int(i16::from_be_bytes(reader.array()?) as i64),
                Prim::Int32 => Value::Int(i32::from_be_bytes(reader.array()?) as i64),
                Prim::Int64 => Value::Int(i64::from_be_bytes(reader.array()?)),
                Prim::Float => Value::Float(f32::from_be_bytes(reader.array()?) as f64),
                Prim::Double => Value::Float(f64::from_be_bytes(reader.array()?)),
                Prim::String => {
                    let length = i32::from_be_bytes(reader.array()?);
                    let length = usize::try_from(length).map_err(|_| anyhow!("negative string length {length}"))?;
                    ensure!(length >= 1, "string length 0 has no terminator");
                    let bytes = reader.take(length)?;
                    ensure!(bytes[length - 1] == 0, "string is not NUL-terminated");
                    Value::Str(bytes[..length - 1].to_vec())
                }
            },
        })
    }

    /// Encodes a top-level message (fingerprint first). Variable lengths are taken from the arrays themselves.
    pub fn encode(&self, name: &str, value: &Value) -> Result<Vec<u8>> {
        let fingerprint = self.fingerprint(name).ok_or_else(|| anyhow!("unknown LCM type {name}"))?;
        let mut out = fingerprint.to_be_bytes().to_vec();
        self.encode_struct(name, value, &mut out)?;
        Ok(out)
    }

    fn encode_struct(&self, name: &str, value: &Value, out: &mut Vec<u8>) -> Result<()> {
        let definition = self.get(name).ok_or_else(|| anyhow!("unknown LCM type {name}"))?;
        let fields = value.fields().ok_or_else(|| anyhow!("{name}: not a struct value"))?;
        ensure!(fields.len() == definition.members.len(), "{name}: {} fields for {} members", fields.len(), definition.members.len());
        // a length member is written as the length of the array that names it
        let mut lengths: HashMap<&str, usize> = HashMap::new();
        for (member, field) in definition.members.iter().zip(fields) {
            if let Some(Dim::Var(length)) = member.dims.first() {
                let element = match member.ty {
                    MemberType::Prim(prim) => prim,
                    MemberType::Struct(_) => Prim::Byte,
                };
                lengths.insert(length, field.len(element).ok_or_else(|| anyhow!("{name}.{}: not an array", member.name))?);
            }
        }
        for (member, field) in definition.members.iter().zip(fields) {
            if let Some(length) = lengths.get(member.name.as_str()) {
                let length = Value::Int(*length as i64);
                self.encode_dims(&member.ty, member.dims.len(), &length, out)?;
                continue;
            }
            self.encode_dims(&member.ty, member.dims.len(), field, out).with_context(|| format!("{name}.{}", member.name))?;
        }
        Ok(())
    }

    fn encode_dims(&self, ty: &MemberType, depth: usize, value: &Value, out: &mut Vec<u8>) -> Result<()> {
        if depth == 0 {
            return self.encode_one(ty, value, out);
        }
        match value {
            Value::Bytes(bytes) | Value::Prims(_, bytes) if depth == 1 => out.extend_from_slice(bytes),
            Value::Array(items) => {
                for item in items {
                    self.encode_dims(ty, depth - 1, item, out)?;
                }
            }
            _ => bail!("expected an array"),
        }
        Ok(())
    }

    fn encode_one(&self, ty: &MemberType, value: &Value, out: &mut Vec<u8>) -> Result<()> {
        let MemberType::Prim(prim) = ty else {
            let MemberType::Struct(name) = ty else { unreachable!() };
            return self.encode_struct(name, value, out);
        };
        let integer = || value.as_i64().ok_or_else(|| anyhow!("expected an integer"));
        match prim {
            Prim::Int8 | Prim::Byte => out.push(integer()? as u8),
            Prim::Boolean => out.push((integer()? != 0) as u8),
            Prim::Int16 => out.extend_from_slice(&(integer()? as i16).to_be_bytes()),
            Prim::Int32 => out.extend_from_slice(&(integer()? as i32).to_be_bytes()),
            Prim::Int64 => out.extend_from_slice(&integer()?.to_be_bytes()),
            Prim::Float => out.extend_from_slice(&(value.as_f64().ok_or_else(|| anyhow!("expected a float"))? as f32).to_be_bytes()),
            Prim::Double => out.extend_from_slice(&value.as_f64().ok_or_else(|| anyhow!("expected a float"))?.to_be_bytes()),
            Prim::String => {
                let Value::Str(bytes) = value else { bail!("expected a string") };
                out.extend_from_slice(&((bytes.len() + 1) as i32).to_be_bytes());
                out.extend_from_slice(bytes);
                out.push(0);
            }
        }
        Ok(())
    }
}

struct Reader<'a> {
    data: &'a [u8],
    offset: usize,
}

impl Reader<'_> {
    fn remaining(&self) -> usize {
        self.data.len() - self.offset
    }

    fn take(&mut self, count: usize) -> Result<&[u8]> {
        ensure!(count <= self.remaining(), "truncated: needs {count} bytes, {} left", self.remaining());
        let slice = &self.data[self.offset..self.offset + count];
        self.offset += count;
        Ok(slice)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N]> {
        Ok(self.take(N)?.try_into().unwrap())
    }
}

// lcm-gen's member hash (lcmgen.c): signed 64-bit, arithmetic shift, chars sign-extended
fn hash_update(value: i64, character: i8) -> i64 {
    ((value << 8) ^ (value >> 55)).wrapping_add(character as i64)
}

fn hash_string_update(mut value: i64, text: &str) -> i64 {
    value = hash_update(value, text.len() as i8);
    for byte in text.bytes() {
        value = hash_update(value, byte as i8);
    }
    value
}

/// Parses the structs in one `.lcm` file (constants are skipped: they are not on the wire or in the hash).
pub fn parse(text: &str) -> Result<Vec<Struct>> {
    let tokens = tokenize(text);
    let mut position = 0;
    let mut package = String::new();
    let mut structs = Vec::new();
    let next = |position: &mut usize| -> Result<String> {
        let token = tokens.get(*position).cloned().ok_or_else(|| anyhow!("unexpected end of file"))?;
        *position += 1;
        Ok(token)
    };
    while position < tokens.len() {
        match next(&mut position)?.as_str() {
            "package" => {
                package = next(&mut position)?;
                ensure!(next(&mut position)? == ";", "expected ; after package");
            }
            "struct" => {
                let name = next(&mut position)?;
                ensure!(next(&mut position)? == "{", "expected {{ after struct {name}");
                let mut members = Vec::new();
                let mut hash: i64 = 0x12345678;
                loop {
                    let token = next(&mut position)?;
                    if token == "}" {
                        break;
                    }
                    if token == "const" {
                        while next(&mut position)? != ";" {}
                        continue;
                    }
                    let ty = match Prim::parse(&token) {
                        Some(prim) => MemberType::Prim(prim),
                        None if token.contains('.') => MemberType::Struct(token.clone()),
                        None => MemberType::Struct(format!("{package}.{token}")),
                    };
                    let member_name = next(&mut position)?;
                    let mut dims = Vec::new();
                    loop {
                        let token = next(&mut position)?;
                        match token.as_str() {
                            ";" => break,
                            "[" => {
                                let size = next(&mut position)?;
                                ensure!(next(&mut position)? == "]", "expected ]");
                                dims.push(match size.parse::<usize>() {
                                    Ok(fixed) => Dim::Fixed(fixed),
                                    Err(_) => Dim::Var(size),
                                });
                            }
                            other => bail!("{name}.{member_name}: unexpected {other:?}"),
                        }
                    }
                    hash = hash_string_update(hash, &member_name);
                    if let MemberType::Prim(prim) = &ty {
                        hash = hash_string_update(hash, prim.lcm_name());
                    }
                    hash = hash_update(hash, dims.len() as i8);
                    for dim in &dims {
                        let (mode, size) = match dim {
                            Dim::Fixed(size) => (0, size.to_string()),
                            Dim::Var(size) => (1, size.clone()),
                        };
                        hash = hash_update(hash, mode);
                        hash = hash_string_update(hash, &size);
                    }
                    members.push(Member { name: member_name, ty, dims });
                }
                structs.push(Struct { name: format!("{package}.{name}"), members, base_hash: hash as u64 });
            }
            other => bail!("unexpected {other:?}"),
        }
    }
    Ok(structs)
}

fn tokenize(text: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut characters = text.chars().peekable();
    let mut current = String::new();
    let flush = |current: &mut String, tokens: &mut Vec<String>| {
        if !current.is_empty() {
            tokens.push(std::mem::take(current));
        }
    };
    while let Some(character) = characters.next() {
        match character {
            '/' if characters.peek() == Some(&'/') => {
                flush(&mut current, &mut tokens);
                for skipped in characters.by_ref() {
                    if skipped == '\n' {
                        break;
                    }
                }
            }
            '/' if characters.peek() == Some(&'*') => {
                flush(&mut current, &mut tokens);
                characters.next();
                let mut previous = ' ';
                for skipped in characters.by_ref() {
                    if previous == '*' && skipped == '/' {
                        break;
                    }
                    previous = skipped;
                }
            }
            '{' | '}' | ';' | '[' | ']' | '=' | ',' => {
                flush(&mut current, &mut tokens);
                tokens.push(character.to_string());
            }
            whitespace if whitespace.is_whitespace() => flush(&mut current, &mut tokens),
            other => current.push(other),
        }
    }
    flush(&mut current, &mut tokens);
    tokens
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fingerprints_match_the_ones_lcm_gen_emits() {
        let registry = registry();
        // from dimos-lcm's generated code (the Controller's msgs.rs carried these)
        for (name, expected) in [
            ("sensor_msgs.Image", [0x53, 0x5c, 0xfa, 0xce, 0x1f, 0x4f, 0x57, 0x17]),
            ("geometry_msgs.Twist", [0x2e, 0x7c, 0x07, 0xd7, 0xcd, 0xf7, 0xe0, 0x27]),
            ("tf2_msgs.TFMessage", [0xc2, 0xb8, 0xa1, 0xc3, 0x3a, 0x89, 0x23, 0xec]),
            ("sensor_msgs.PointCloud2", [0xf5, 0xeb, 0x3d, 0xa1, 0xc2, 0x85, 0x31, 0x75]),
            ("nav_msgs.Odometry", [0x94, 0xe1, 0x7f, 0x94, 0x64, 0x8c, 0xf0, 0xe4]),
            ("sensor_msgs.CameraInfo", [0xaf, 0xb5, 0x01, 0xb0, 0x07, 0x07, 0xa0, 0x2a]),
            ("std_msgs.String", [0x21, 0xbf, 0x37, 0x09, 0x9b, 0x9d, 0x5e, 0x15]),
        ] {
            assert_eq!(registry.fingerprint(name).unwrap(), u64::from_be_bytes(expected), "{name}");
        }
    }

    #[test]
    fn every_vendored_type_parses_and_fingerprints() {
        let registry = registry();
        assert!(registry.structs.len() >= 170);
        for name in registry.structs.keys() {
            assert!(registry.fingerprint(name).is_some(), "{name}");
        }
    }

    #[test]
    fn decode_then_encode_is_the_identity() {
        let registry = registry();
        let mut image = registry.fingerprint("sensor_msgs.Image").unwrap().to_be_bytes().to_vec();
        image.extend_from_slice(&6i32.to_be_bytes()); // data_length
        image.extend_from_slice(&7i32.to_be_bytes()); // seq
        image.extend_from_slice(&12i32.to_be_bytes());
        image.extend_from_slice(&34i32.to_be_bytes());
        image.extend_from_slice(&4i32.to_be_bytes());
        image.extend_from_slice(b"cam\0");
        image.extend_from_slice(&1i32.to_be_bytes()); // height
        image.extend_from_slice(&2i32.to_be_bytes()); // width
        image.extend_from_slice(&5i32.to_be_bytes());
        image.extend_from_slice(b"rgb8\0");
        image.push(0);
        image.extend_from_slice(&6i32.to_be_bytes()); // step
        image.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
        let value = registry.decode("sensor_msgs.Image", &image).unwrap();
        assert_eq!(value.fields().unwrap()[7], Value::Bytes(vec![1, 2, 3, 4, 5, 6]));
        assert_eq!(registry.encode("sensor_msgs.Image", &value).unwrap(), image);
        assert!(registry.decode("sensor_msgs.Imu", &image).is_err());
        assert!(registry.decode("sensor_msgs.Image", &image[..image.len() - 1]).is_err());
    }
}
