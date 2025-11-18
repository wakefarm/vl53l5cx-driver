use std::env;
use std::path::PathBuf;

fn main() {
    // Directory containing the ST ULD source files
    let driver_dir = PathBuf::from("src/VL53L5CX_ULD_API");

    // List of C source files to compile
    let c_sources = vec![
        driver_dir.join("vl53l5cx_api.c"),
        driver_dir.join("vl53l5cx_buffers.c"),
        driver_dir.join("vl53l5cx_plugin_motion_indicator.c"),
        driver_dir.join("vl53l5cx_plugin_xtalk.c"),
    ];

    // Compile the C sources into a static library
    cc::Build::new()
        .files(c_sources)
        .include(&driver_dir)
        .define("VL53L5CX_NB_TARGET_PER_ZONE", "1")
        // This tells the C driver to use the high-level platform functions.
        .define("USE_ULP_PLATFORM", "1")
        .compile("vl53l5cx_driver");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/VL53L5CX_ULD_API/vl53l5cx_api.h");

    // Generate the Rust bindings using bindgen
    let bindings = bindgen::Builder::default()
        .header(driver_dir.join("vl53l5cx_api.h").to_str().unwrap())
        // Also define the macros for bindgen's C parser.
        .clang_arg("-DVL53L5CX_NB_TARGET_PER_ZONE=1")
        .clang_arg("-DUSE_ULP_PLATFORM=1")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Whitelist the functions, types, and constants we want to use.
        .allowlist_function("vl53l5cx_.*")
        .allowlist_type("VL53L5CX_.*")
        .allowlist_var("VL53L5CX_.*") // This will include VL53L5CX_STATUS_OK
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
