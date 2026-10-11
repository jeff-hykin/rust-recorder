//! Deterministic sample messages for any vendored LCM type (tests and the reference-decoder fixtures).

use crate::lcm::{self, Dim, MemberType, Prim, Value};

/// A sample value of `lcm_type` whose every field is set from `seed`: valid times, short arrays, finite floats that
/// float32 holds exactly, ASCII strings.
pub fn sample_value(lcm_type: &str, seed: u64) -> Value {
    let mut state = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    sample_struct(lcm_type, &mut state, 0)
}

fn next(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    *state >> 33
}

fn sample_struct(lcm_type: &str, state: &mut u64, depth: usize) -> Value {
    let definition = lcm::registry().get(lcm_type).unwrap_or_else(|| panic!("unknown {lcm_type}"));
    let is_time = matches!(lcm_type, "std_msgs.Time" | "builtin_interfaces.Time" | "std_msgs.Duration" | "builtin_interfaces.Duration");
    let mut fields = Vec::new();
    for (index, member) in definition.members.iter().enumerate() {
        if is_time {
            fields.push(Value::Int(if index == 0 { 1_700_000_000 + (next(state) % 1000) as i64 } else { (next(state) % 1_000_000_000) as i64 }));
            continue;
        }
        let value = match member.dims.first() {
            None => sample_one(&member.ty, state, depth),
            Some(dim) => {
                let count = match dim {
                    Dim::Fixed(size) => *size,
                    Dim::Var(_) => if depth > 2 { 0 } else { 1 + (next(state) % 3) as usize },
                };
                sample_array(&member.ty, count, state, depth)
            }
        };
        fields.push(value);
    }
    // a length member says how long its array is
    for (index, member) in definition.members.iter().enumerate() {
        if let Some(Dim::Var(length)) = member.dims.first() {
            let element = match member.ty { MemberType::Prim(prim) => prim, MemberType::Struct(_) => Prim::Byte };
            let count = fields[index].len(element).unwrap();
            let at = definition.member_index(length).unwrap();
            fields[at] = Value::Int(count as i64);
        }
    }
    Value::Struct(fields)
}

fn sample_array(ty: &MemberType, count: usize, state: &mut u64, depth: usize) -> Value {
    match ty {
        MemberType::Prim(Prim::Byte | Prim::Int8) => Value::Bytes((0..count).map(|_| next(state) as u8).collect()),
        MemberType::Prim(Prim::Boolean) => Value::Bytes((0..count).map(|_| (next(state) % 2) as u8).collect()),
        MemberType::Prim(Prim::String) => Value::Array((0..count).map(|_| sample_one(ty, state, depth)).collect()),
        MemberType::Prim(prim) => {
            let mut bytes = Vec::new();
            for _ in 0..count {
                bytes.extend(prim_bytes(*prim, state));
            }
            Value::Prims(*prim, bytes)
        }
        MemberType::Struct(name) => Value::Array((0..count).map(|_| sample_struct(name, state, depth + 1)).collect()),
    }
}

fn prim_bytes(prim: Prim, state: &mut u64) -> Vec<u8> {
    let raw = next(state);
    match prim {
        Prim::Int16 => (raw as i16).to_be_bytes().to_vec(),
        Prim::Int32 => (raw as i32).to_be_bytes().to_vec(),
        Prim::Int64 => ((raw as i64) << 20 | raw as i64).to_be_bytes().to_vec(),
        Prim::Float => ((raw % 4096) as f32 / 4.0 - 512.0).to_be_bytes().to_vec(),
        Prim::Double => ((raw % 1_000_000) as f64 / 8.0 - 60_000.0).to_be_bytes().to_vec(),
        _ => vec![raw as u8],
    }
}

fn sample_one(ty: &MemberType, state: &mut u64, depth: usize) -> Value {
    match ty {
        MemberType::Struct(name) => sample_struct(name, state, depth + 1),
        MemberType::Prim(prim) => match prim {
            Prim::String => Value::Str(format!("s{}", next(state) % 1000).into_bytes()),
            Prim::Boolean => Value::Bool(next(state) % 2 == 1),
            Prim::Byte => Value::Int((next(state) % 256) as i64),
            Prim::Int8 => Value::Int((next(state) % 256) as u8 as i8 as i64),
            Prim::Float | Prim::Double => {
                let bytes = prim_bytes(*prim, state);
                Value::Float(if *prim == Prim::Float { f32::from_be_bytes(bytes.try_into().unwrap()) as f64 } else { f64::from_be_bytes(bytes.try_into().unwrap()) })
            }
            other => {
                let bytes = prim_bytes(*other, state);
                Value::Int(match other {
                    Prim::Int16 => i16::from_be_bytes(bytes.try_into().unwrap()) as i64,
                    Prim::Int32 => i32::from_be_bytes(bytes.try_into().unwrap()) as i64,
                    _ => i64::from_be_bytes(bytes.try_into().unwrap()),
                })
            }
        },
    }
}
