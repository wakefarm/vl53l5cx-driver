use std::env;
use std::path::PathBuf;

fn main() {
    // Tell rustc that we are creating a custom `cfg` flag. This is required to
    // prevent a warning about an unexpected `cfg` condition name.
    println!("cargo:rustc-check-cfg=cfg(motion_or_thresholds)");

    // Directory containing the ST ULD source files
    let driver_dir = PathBuf::from("src").join("VL53L5CX_ULD_API");

    // Start with the base API file, which is always required.
    let mut c_sources = vec![driver_dir.join("src").join("vl53l5cx_api.c")];
    let mut cc_build = cc::Build::new();

    // Determine the number of targets based on active features.
    let mut nb_targets = "1"; // Default
    if env::var("CARGO_FEATURE_TARGETS_4").is_ok() {
        nb_targets = "4";
    } else if env::var("CARGO_FEATURE_TARGETS_3").is_ok() {
        nb_targets = "3";
    } else if env::var("CARGO_FEATURE_TARGETS_2").is_ok() {
        nb_targets = "2";
    }
    // Note: If multiple `targets-*` features are enabled, the highest number will be used.
    // For true mutual exclusion, a more complex check could be added here to panic the build.

    let mut bindgen_builder = bindgen::Builder::default();

    // --- Feature: xtalk ---
    if env::var("CARGO_FEATURE_XTALK").is_ok() {
        println!("cargo:rustc-cfg=feature=\"xtalk\"");
        c_sources.push(driver_dir.join("src").join("vl53l5cx_plugin_xtalk.c"));
        cc_build.define("VL53L5CX_USE_XTALK", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L5CX_USE_XTALK=1");
    }

    // --- Feature: motion & thresholds ---
    // The motion indicator plugin contains code for both motion and detection thresholds.
    if env::var("CARGO_FEATURE_MOTION").is_ok() || env::var("CARGO_FEATURE_THRESHOLDS").is_ok() {
        println!("cargo:rustc-cfg=motion_or_thresholds");
        c_sources.push(driver_dir.join("src").join("vl53l5cx_plugin_motion_indicator.c"));
        cc_build.define("VL53L5CX_USE_MOTION_INDICATOR", "1");
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L5CX_USE_MOTION_INDICATOR=1");
    }

    // If the motion feature is disabled, we must tell the C header to remove the
    // `motion_indicator` field from the VL53L5CX_ResultsData struct to keep layouts compatible.
    if env::var("CARGO_FEATURE_MOTION").is_err() {
        bindgen_builder = bindgen_builder.clang_arg("-DVL53L5CX_DISABLE_MOTION_INDICATOR");
    }

    // Compile the C sources into a static library
    cc_build
        .files(c_sources)
        .include(driver_dir.join("inc"))
        // Configure driver settings.
        .define("VL53L5CX_NB_TARGET_PER_ZONE", nb_targets)
        // This tells the C driver to use the high-level platform functions.
        .define("USE_ULP_PLATFORM", "1")
        .compile("vl53l5cx_driver");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/VL53L5CX_ULD_API/inc/vl53l5cx_api.h");

    // Generate the Rust bindings using bindgen
    let bindings = bindgen_builder
        .header(driver_dir.join("inc").join("vl53l5cx_api.h").to_str().unwrap())
        // Explicitly include plugin headers so bindgen can find all types.
        .header(driver_dir.join("inc").join("vl53l5cx_plugin_xtalk.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l5cx_plugin_motion_indicator.h").to_str().unwrap())
        .header(driver_dir.join("inc").join("vl53l5cx_plugin_detection_thresholds.h").to_str().unwrap())
        // Add the include path for bindgen's clang parser.
        .clang_arg(format!("-I{}", driver_dir.join("inc").display()))
        // Also define the macros for bindgen's C parser.
        .clang_arg(format!("-DVL53L5CX_NB_TARGET_PER_ZONE={}", nb_targets))
        .clang_arg("-DUSE_ULP_PLATFORM=1")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Whitelist the functions, types, and constants we want to use.
        .allowlist_function("vl53l5cx_.*")
        .allowlist_type("VL53L5CX_.*")
        .allowlist_var("VL53L5CX_.*")
        .generate()
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
