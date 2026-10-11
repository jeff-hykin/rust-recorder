// Embeds the vendored message definitions (msgs/lcm/*.lcm, msgs/ros2/<package>/*.msg) as static tables.

use std::fmt::Write;
use std::path::Path;

fn sorted_files(directory: &Path, extension: &str) -> Vec<std::path::PathBuf> {
    let mut files: Vec<_> = std::fs::read_dir(directory)
        .unwrap_or_else(|error| panic!("reading {}: {error}", directory.display()))
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|end| end == extension))
        .collect();
    files.sort();
    files
}

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("msgs");
    println!("cargo:rerun-if-changed=msgs");
    let mut out = String::from("pub(crate) static LCM_DEFINITIONS: &[(&str, &str)] = &[\n");
    for path in sorted_files(&root.join("lcm"), "lcm") {
        let name = path.file_stem().unwrap().to_str().unwrap();
        writeln!(out, "    ({name:?}, include_str!({:?})),", path.display().to_string()).unwrap();
    }
    out.push_str("];\n\npub(crate) static ROS2_DEFINITIONS: &[(&str, &str, &str)] = &[\n");
    let mut packages: Vec<_> = std::fs::read_dir(root.join("ros2"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.is_dir())
        .collect();
    packages.sort();
    for package in packages {
        let package_name = package.file_name().unwrap().to_str().unwrap().to_string();
        for path in sorted_files(&package, "msg") {
            let name = path.file_stem().unwrap().to_str().unwrap();
            writeln!(out, "    ({package_name:?}, {name:?}, include_str!({:?})),", path.display().to_string()).unwrap();
        }
    }
    out.push_str("];\n");
    std::fs::write(Path::new(&std::env::var("OUT_DIR").unwrap()).join("definitions.rs"), out).unwrap();
}
