use std::env;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    #[cfg(feature = "matekh743")]
    std::fs::copy("board/src/matekh743/memory.x", std::path::PathBuf::from(out_dir.as_str()).join("memory.x"),).unwrap();
    #[cfg(feature = "f407-disco")]
    std::fs::copy("board/src/disco407/memory.x", std::path::PathBuf::from(out_dir.as_str()).join("memory.x"),).unwrap();

    println!("cargo:rustc-link-search={}", out_dir);

    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

}
