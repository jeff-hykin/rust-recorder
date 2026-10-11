//! Prints how every vendored dimos-lcm type is recorded in a ROS 2 MCAP (the README's coverage table).

use rust_recorder::ros2::{mapping, Mapping};

fn main() {
    let mut names: Vec<&String> = rust_recorder::lcm::registry().structs.keys().collect();
    names.sort();
    let markdown = std::env::args().any(|argument| argument == "--markdown");
    for name in names {
        match mapping(name) {
            Mapping::Cdr(map) if map.defaulted.is_empty() => println!("{}", if markdown { format!("| `{name}` | `{}` | CDR |", map.ros) } else { format!("CDR       {name} -> {}", map.ros) }),
            Mapping::Cdr(map) => println!("{}", if markdown { format!("| `{name}` | `{}` | CDR (zero: {}) |", map.ros, map.defaulted.join(", ")) } else { format!("CDR+zero  {name} -> {} (zero: {})", map.ros, map.defaulted.join(", ")) }),
            Mapping::Unmapped(reason) => println!("{}", if markdown { format!("| `{name}` | — | raw LCM: {reason} |") } else { format!("LCM       {name}: {reason}") }),
        }
    }
}
