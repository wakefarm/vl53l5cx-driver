# VL53L5CX-FFI

[![Crates.io](https://img.shields.io/crates/v/vl53l5cx-ffi.svg)](https://crates.io/crates/vl53l5cx-ffi)
[![Docs.rs](https://docs.rs/vl53l5cx-ffi/badge.svg)](https://docs.rs/vl53l5cx-ffi)
[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](LICENSE-MIT)
[![Mermaid Support](https://img.shields.io/badge/VS%20Code-Mermaid%20Support-blue?logo=visualstudiocode)](https://marketplace.visualstudio.com/items?itemName=bierner.markdown-mermaid)

A platform-agnostic Rust driver for the **STMicroelectronics VL53L5CX** Time-of-Flight (ToF) multi-zone ranging sensor.

This crate acts as a high-level wrapper around the official ST Ultra Lite Driver (ULD), utilizing `embedded-hal` traits to allow usage on:
* **Microcontrollers:** STM32, ESP32, nRF52, RP2040, etc. (`no_std`)
* **Linux:** Raspberry Pi, Jetson Nano, etc. (`std`)
* **Windows / macOS:** Via USB-to-I2C adapters (FT232H, MCP2221) using crates like `ftdi-embedded-hal`.

## Features

- **Platform Agnostic**: Built on `embedded-hal` (I2C) traits.
- **FFI Wrapper**: Statically links the official ST VL53L5CX ULD C-driver.
- **Multi-Zone Ranging**: Access to 4x4 or 8x8 zones functionality provided by the ULD.
- **Safe API**: Exposes a Rust-friendly API while handling the unsafe FFI calls internally.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
vl53l5cx-ffi = "0.1.0"
```

### Requirements
* **Rust:** Stable toolchain
* **C Compiler:** A compatible C compiler must be installed (e.g., `gcc`, `clang`, or MSVC).
* **LLVM/Clang:** Required by `bindgen` to generate bindings.

## Usage

Note that while the crate name uses dashes (`-`), the library is imported using underscores (`_`) in your Rust code.

```rust
use vl53l5cx_ffi::{Vl53l5cx, Resolution, VL53L5CX_ResultsData, platform::PlatformError};
use embedded_hal::{i2c::{I2c, SevenBitAddress}, delay::DelayNs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Initialize your platform-specific I2C and Delay peripherals
    // This is just an example; you would use your specific HAL implementation.
    let i2c_dev: impl I2c<SevenBitAddress> = todo!(); 
    let delay_dev: impl DelayNs = todo!();
    
    // 2. Box the peripherals for type erasure
    let i2c_bus: Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send> = 
        Box::new(i2c_dev.map_err(PlatformError::from));
    let delay: Box<dyn DelayNs + Send> = Box::new(delay_dev);

    // 3. Initialize the sensor driver
    let mut sensor = Vl53l5cx::new(i2c_bus, delay, 0x29)?;
    
    // 4. Check communication and configure
    if sensor.is_alive()? {
        sensor.set_resolution(Resolution::Res8x8)?;
        sensor.set_ranging_frequency_hz(15)?;
        
        sensor.start_ranging()?;
        
        loop {
            // Poll for data ready
            if sensor.check_data_ready()? {
                let results: VL53L5CX_ResultsData = sensor.get_ranging_data()?;
                // Process ranging data...
                // println!("Zone 0 Distance: {} mm", results.distance_mm[0]);
            }
        }
    }
    
    Ok(())
}
```

# API Reference & Feature Guide

The `vl53l5cx-ffi` crate exposes a safe Rust API over the STMicroelectronics Ultra Lite Driver (ULD). While the core ranging functionality is always available, advanced features like motion detection, crosstalk calibration, and multi-target tracking must be enabled at compile time to optimize memory usage.

## 1. Feature Flags (Compile-Time Configuration)

To optimize RAM and binary size, this crate uses Cargo features to conditionally compile specific ULD plugins. You must enable the relevant features in your `Cargo.toml`.

```toml
[dependencies]
vl53l5cx-ffi = { version = "0.1.0", features = ["xtalk", "motion", "targets-4"] }
```

| Feature | Description | Enables Functions |
| :--- | :--- | :--- |
| **`xtalk`** | Crosstalk calibration support. Essential if using a cover glass. | `calibrate_xtalk`, `get_caldata_xtalk`, `set_caldata_xtalk` |
| **`motion`** | Motion indicator plugin. Detects movement per zone. | `motion_indicator_init`, `motion_indicator_set_distance_motion` |
| **`thresholds`**| Detection thresholds. Configures interrupt triggers based on distance/signal. | `set_detection_thresholds` |

### Target Capacity Configuration
These features control the maximum number of targets tracked per zone. Select **only one**. Higher target counts increase RAM usage.

| Feature | Description | Memory Impact |
| :--- | :--- | :--- |
| **`targets-1`** | **(Default)** Tracks the single strongest target. | Lowest RAM. |
| **`targets-2`** | Tracks up to 2 targets per zone. | Medium RAM. |
| **`targets-3`** | Tracks up to 3 targets per zone. | High RAM. |
| **`targets-4`** | Tracks up to 4 targets per zone. | Max RAM. |

## 2. Rust Idioms & Data Structures

This driver utilizes idiomatic Rust enums and best practices to ensure safe and performant access to the sensor.

### Handling Flat Arrays (Best Practice)
The driver returns ranging data as a flat array (e.g., `[i16; 64]` for 1 target/zone) to match the sensor's memory layout and avoid allocation overhead. To access this data as a 2D grid (8x8) without sacrificing performance, use the `.chunks(8)` iterator.

```rust
let results = sensor.get_ranging_data()?;

// Iterate over the 8x8 grid row by row
for (y, row) in results.distance_mm.chunks(8).enumerate() {
    for (x, distance) in row.iter().enumerate() {
        println!("Zone ({}, {}): {} mm", x, y, distance);
    }
}
```

### Key Enums
* **`Resolution`**: 
    * `Res4x4`: 16 zones (4x4 grid). Max frequency 60Hz.
    * `Res8x8`: 64 zones (8x8 grid). Max frequency 15Hz.
* **`RangingMode`**: 
    * `Continuous`: Sensor streams frames as fast as possible.
    * `Autonomous`: Sensor sleeps between frames (low power).
* **`PowerMode`**: 
    * `Sleep`: Firmware retained, no ranging, lowest power.
    * `Wakeup`: Ready to range.
* **`TargetOrder`**: `Closest` (default) or `Strongest`.

## 3. Function Reference

All functions return `Result<T, DriverError>`. The return type listed below refers to the `Ok(T)` variant.

### Core Management
Lifecycle and power management functions.

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`new`** | `i2c`, `delay`, `addr`: `u8` | `Vl53l5cx` | **Constructor.** Consumes peripherals, uploads firmware (~90KB), and boots sensor. `addr` is typically `0x29`. |
| **`is_alive`** | None | `bool` | Returns `true` if Device ID is `0xF0` and Revision is `0x02`. |
| **`set_i2c_address`** | `addr`: `u8` | `()` | Changes 7-bit I2C address. Resets to default on power cycle. |
| **`set_power_mode`** | `mode`: [`PowerMode`](#key-enums) | `()` | Toggles between `Sleep` (low power) and `Wakeup`. |
| **`enable_internal_cp`** | None | `()` | Enables internal VCSEL charge pump (default). |
| **`disable_internal_cp`**| None | `()` | Disables charge pump. **Only use if AVDD is 3.3V**. |

### Ranging Configuration
Functions to configure the sensing output. **These must be called before `start_ranging()`.**

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`set_resolution`** | `res`: [`Resolution`](#key-enums) | `()` | Sets grid size to 4x4 or 8x8. |
| **`set_ranging_frequency_hz`**| `hz`: `u8` | `()` | Sets frame rate. Max 60Hz (4x4) or 15Hz (8x8). |
| **`set_ranging_mode`** | `mode`: [`RangingMode`](#key-enums) | `()` | Selects `Continuous` or `Autonomous` (low power). |
| **`set_integration_time_ms`**| `ms`: `u32` | `()` | Sets exposure time (2-1000ms). Must be < `1/frequency`. |
| **`set_target_order`** | `order`: [`TargetOrder`](#key-enums)| `()` | Sorts targets by `Closest` or `Strongest`. |
| **`set_sharpener_percent`** | `pct`: `u8` | `()` | Configures edge sharpening filter (0-99%). |
| **`set_vhv_repeat_count`** | `cnt`: `u32` | `()` | Sets temp calib interval. `0` disables auto-cal. |

### Ranging Control & Data
Functions to control the active measurement loop.

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`start_ranging`** | None | `()` | Begins measurement session. Settings are locked. |
| **`stop_ranging`** | None | `()` | Ends measurement session, allowing config changes. |
| **`check_data_ready`** | None | `bool` | Non-blocking check. Returns `true` if new data is ready. |
| **`get_ranging_data`** | None | [`ResultsData`](#vl53l5cx_resultsdata) | Reads latest measurement block. |

### Plugin: Crosstalk Calibration
*Requires `features = ["xtalk"]`*

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`calibrate_xtalk`** | `refl`: `u16`, `dist`: `u16`, `samples`: `u8` | `()` | Runs calibration sequence with a target of known reflectance/distance. |
| **`get_caldata_xtalk`** | None | `Vec<u8>` | Downloads calibration data to save to persistent storage. |
| **`set_caldata_xtalk`** | `data`: `&[u8]` | `()` | Uploads previously saved calibration data. |

### Plugin: Motion Indicator
*Requires `features = ["motion"]`*

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`motion_indicator_init`** | `cfg`: `&mut` [`MotionConfig`](#vl53l5cx_motion_configuration), `res`: `u8` | `()` | Initializes motion struct with resolution (16 or 64). |
| **`motion_indicator_set_distance_motion`**| `cfg`: `&mut` [`MotionConfig`](#vl53l5cx_motion_configuration), `min`: `u16`, `max`: `u16` | `()` | Sets spatial window (mm) for motion detection. |

### Plugin: Detection Thresholds
*Requires `features = ["thresholds"]`*

| Function | Arguments | Return | Description |
| :--- | :--- | :--- | :--- |
| **`set_detection_thresholds`** | `cfg`: `&mut` [`DetectionThresholds`](#vl53l5cx_detectionthresholds) | `()` | Uploads threshold logic rules for interrupts. |

---

## 4. Data Structures & Usage

The API exposes raw C-compatible structures via the `bindings` module.

### `VL53L5CX_ResultsData`
Returned by `get_ranging_data()`. This structure holds the sensor output. Arrays are flattened (1D); see "Handling Flat Arrays" above.

**Per-Zone Fields (Common to all targets in a zone):**
| Field | Type | Description |
| :--- | :--- | :--- |
| `ambient_per_spad` | `[u32]` | Ambient noise level (kcps/SPAD). |
| `nb_target_detected` | `[u8]` | Number of valid targets detected in this zone. |
| `nb_spads_enabled` | `[u32]` | Number of SPADs activated for this measurement. |

**Per-Target Fields (Repeated for each target):**
*Note: If `targets-1` is used, the array size matches the resolution (16 or 64). If `targets-4` is used, the array size is `resolution * 4`.*
| Field | Type | Description |
| :--- | :--- | :--- |
| `distance_mm` | `[i16]` | Measured distance to the target in millimeters. |
| `target_status` | `[u8]` | Reliability confidence. **5** and **9** indicate valid, high-confidence ranging. **255** indicates no target. |
| `signal_per_spad` | `[u32]` | Signal strength (kcps/SPAD). |
| `range_sigma_mm` | `[u16]` | Estimated standard deviation (jitter) of the measurement in mm. |
| `reflectance` | `[u8]` | Estimated target reflectance (0-100%). |

### `VL53L5CX_DetectionThresholds`
Used with `set_detection_thresholds`. Represents a **single** logical rule.

**Struct Fields:**
| Field | Type | Description |
| :--- | :--- | :--- |
| `zone_num` | `u8` | The Zone ID (0-63) this rule applies to. |
| `measurement` | `u8` | The data source to check. See constants below. |
| `type` | `u8` | The comparison logic (Window, Min, Max). See constants below. |
| `mathematic_operation`| `u8` | Logic for combining multiple rules on the same zone (`OR`, `AND`). |
| `param_low_thresh` | `i32` | Lower bound value (units depend on `measurement`). |
| `param_high_thresh` | `i32` | Upper bound value (units depend on `measurement`). |

**Measurement Constants:**
Available in `vl53l5cx_ffi::bindings`:
* `VL53L5CX_DISTANCE_MM`: Distance (mm)
* `VL53L5CX_SIGNAL_PER_SPAD_KCPS`: Signal strength
* `VL53L5CX_NB_TARGET_DETECTED`: Target count
* `VL53L5CX_MOTION_INDICATOR`: Motion intensity

**Window Type Constants:**
* `VL53L5CX_IN_WINDOW`: Trigger if value is between Low and High thresholds.
* `VL53L5CX_OUT_OF_WINDOW`: Trigger if value is outside Low/High.
* `VL53L5CX_LESS_THAN_EQUAL_MIN_CHECKER`: Trigger if value <= Low.
* `VL53L5CX_GREATER_THAN_MAX_CHECKER`: Trigger if value > High.

### `VL53L5CX_Motion_Configuration`
Used to configure the Motion Indicator plugin.

| Field | Description |
| :--- | :--- |
| `map_id` | `[i8; 64]` | Mapping of zones to motion indicator fields. Initialized automatically based on resolution. |
| `ref_bin_offset` | `i32` | Internal offset calculated from Min/Max distance settings. |
| `detection_threshold` | `u32` | Threshold for movement sensitivity. |

## Development Notes

### Viewing the Architecture Diagram
The architecture diagram in this README is created using Mermaid. To view it in Visual Studio Code's markdown preview, you will need to install the **Markdown Preview Mermaid Support** extension.


## ⚠️ License & Legal Disclaimer

### Rust Wrapper Code
The Rust source code in this crate is dual-licensed under either:

* **MIT License** ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
* **Apache License, Version 2.0** ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

at your option.

### STMicroelectronics Driver Code
This crate statically links and redistributes the **STMicroelectronics VL53L5CX Ultra Lite Driver**, which is located in the source tree.

**The STMicroelectronics ULD software is subject to the proprietary STMicroelectronics license terms (SLA0044).**

> **Important:** Usage of the underlying C driver is strictly limited to use with **STMicroelectronics products** (specifically the VL53L5CX sensor). By using this crate, you agree to the terms of the STMicroelectronics software license found in the `LICENSE-ST` file included in this repository.