use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Choose memory script based on feature
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let _out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let is_nucleo = env::var("CARGO_FEATURE_NUCLEO").is_ok();
    let memory_src = if is_nucleo {
        manifest_dir.join("memory-nucleo.x")
    } else {
        manifest_dir.join("memory-qemu.x")
    };

    // Write memory.x into the crate root so cortex-m-rt's link.x INCLUDE can find it via link search path.
    let root_memory = manifest_dir.join("memory.x");
    fs::copy(&memory_src, &root_memory).expect("copy memory.x -> crate root");

    // Add crate root to linker search path so INCLUDE memory.x resolves.
    println!("cargo:rustc-link-search={}", manifest_dir.display());
    println!("cargo:rerun-if-changed={}", memory_src.display());
}
