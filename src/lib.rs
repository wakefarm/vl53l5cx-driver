#![cfg_attr(not(feature = "std"), no_std)]

use embedded_hal::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};
use core::ffi::c_void;

#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(not(feature = "std"))]
use alloc::boxed::Box;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

// Include the bindgen-generated bindings.
// We wrap the `include!` in a module to give the generated code its own namespace.
pub mod bindings {
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(dead_code)]
    // This file is created in the OUT_DIR directory during the build process.
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

// Re-export the most commonly used structs for easier access.
pub use bindings::{VL53L5CX_ResultsData, VL53L5CX_Motion_Configuration, VL53L5CX_DetectionThresholds};

/// The size of the Xtalk calibration data buffer.
pub const VL53L5CX_XTALK_BUFFER_SIZE: usize = 776;
// Include the platform glue module.
pub mod platform;

#[derive(Debug)]
pub enum SensorStatus {
    Ok,
    TimeoutError,
    CorruptedFrame,
    CrcCsumFailed,
    XtalkFailed,
    McuError,
    InvalidParam,
    Unknown(u8),
}

impl From<u8> for SensorStatus {
    fn from(code: u8) -> Self {
        match code {
            0 => SensorStatus::Ok,
            1 => SensorStatus::TimeoutError,
            2 => SensorStatus::CorruptedFrame,
            3 => SensorStatus::CrcCsumFailed,
            4 => SensorStatus::XtalkFailed,
            66 => SensorStatus::McuError,
            127 => SensorStatus::InvalidParam,
            c => SensorStatus::Unknown(c),
        }
    }
}

#[derive(Debug)]
#[repr(u8)]
pub enum Resolution {
    Res4x4 = 16,
    Res8x8 = 64,
}

#[derive(Debug)]
#[repr(u8)]
pub enum TargetOrder {
    Closest = 1,
    Strongest = 2,
}

#[derive(Debug)]
#[repr(u8)]
pub enum PowerMode {
    Sleep = 0,
    Wakeup = 1,
}

#[derive(Debug)]
#[repr(u8)]
pub enum RangingMode {
    Continuous = 1,
    Autonomous = 3,
}

#[derive(Debug)]
pub enum DriverError {
    /// An I2C communication error occurred in the platform layer.
    /// The C driver reports this as status 255.
    I2c,
    /// The sensor returned a non-zero status code, indicating an error.
    Sensor(SensorStatus),
}

impl core::fmt::Display for DriverError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DriverError::I2c => write!(f, "I2C communication error"),
            DriverError::Sensor(status) => write!(f, "Sensor driver error: {:?}", status),
        }
    }
}

/// Represents a VL53L5CX sensor instance.
pub struct Vl53l5cx {
    /// The ST driver configuration struct. This is the main handle used by the C API.
    /// It's boxed to give it a stable memory address.
    config: Box<bindings::VL53L5CX_Configuration>,
    // We hold the raw pointer to the PlatformResources to deallocate it in Drop.
    platform_resources_ptr: *mut platform::PlatformResources,
}

impl Vl53l5cx {
    /// Creates a new driver instance and initializes the sensor.
    ///
    /// This function takes ownership of the I2C bus, initializes the platform layer,
    /// and then calls the C driver's `vl53l5cx_init` function.
    pub fn new(
        i2c_bus: Box<dyn I2c<SevenBitAddress, Error = platform::PlatformError> + Send>,
        delay: Box<dyn DelayNs + Send>,
        address: u8,
    ) -> Result<Self, DriverError> {
        // 1. Package the I2C bus and Delay provider into the PlatformResources struct.
        //    This struct will be our "context pointer" passed to the C driver.
        let platform_resources = Box::new(platform::PlatformResources {
            i2c: i2c_bus,
            delay,
        });
        // Convert the Box into a raw pointer. This gives us a stable memory address
        // that we can pass to C, while preventing the Box from being dropped.
        let platform_resources_ptr = Box::into_raw(platform_resources);

        let mut config: Box<bindings::VL53L5CX_Configuration> =
            Box::new(unsafe { core::mem::zeroed() });

        // 2. The C driver's platform struct has a `p_com` field that we can use
        //    to store a pointer to our PlatformResources struct.
        let platform_ptr = &mut config.platform as *mut _ as *mut platform::VL53L5CX_Platform;
        unsafe {
            (*platform_ptr).p_com = platform_resources_ptr as *mut c_void;
            (*platform_ptr).address = address as u16;
        }

        // 3. Call the C API to initialize the sensor.
        let status = unsafe { bindings::vl53l5cx_init(config.as_mut()) };

        // 4. Check the status code. If it fails, we must clean up the allocated
        //    platform resources to prevent a memory leak.
        if let Err(e) = Self::check_status(status) {
            // SAFETY: We must reconstruct the box so it is dropped properly if initialization fails.
            unsafe { let _ = Box::from_raw(platform_resources_ptr); }
            return Err(e);
        }

        Ok(Self {
            config,
            platform_resources_ptr,
        })
    }

    /// Checks if the sensor is alive and ready on the I2C bus.
    pub fn is_alive(&mut self) -> Result<bool, DriverError> {
        let mut is_alive: u8 = 0;
        let status = unsafe {
            bindings::vl53l5cx_is_alive(self.config.as_mut(), &mut is_alive)
        };

        Self::check_status(status).map(|_| is_alive != 0)
    }

    /// Sets a new I2C address for the sensor.
    ///
    /// This function changes the I2C address of the sensor and updates the
    /// address stored in the driver configuration. The new address will be
    /// used for all subsequent communications.
    ///
    /// # Arguments
    ///
    /// * `address` - The new 7-bit I2C address.
    pub fn set_i2c_address(&mut self, address: u8) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_set_i2c_address(self.config.as_mut(), address as u16) };
        Self::check_status(status)
    }

    /// Sets the sensor resolution.
    ///
    /// The resolution can be either 4x4 (`VL53L5CX_RESOLUTION_4X4`) or
    /// 8x8 (`VL53L5CX_RESOLUTION_8X8`).
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_resolution(self.config.as_mut(), resolution as u8)
        };
        Self::check_status(status)
    }

    /// Sets the ranging frequency in Hz.
    ///
    /// The frequency must be between 1 and 60 Hz.
    /// Note that the maximum frequency for 8x8 resolution is 15Hz.
    pub fn set_ranging_frequency_hz(&mut self, frequency: u8) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_ranging_frequency_hz(self.config.as_mut(), frequency)
        };
        Self::check_status(status)
    }

    /// Gets the current sharpener percentage.
    ///
    /// The sharpener is a post-processing filter that can help to separate targets.
    pub fn get_sharpener_percent(&mut self) -> Result<u8, DriverError> {
        let mut sharpener_percent: u8 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_sharpener_percent(self.config.as_mut(), &mut sharpener_percent)
        };
        Self::check_status(status).map(|_| sharpener_percent)
    }

    /// Sets the sharpener percentage.
    ///
    /// The sharpener is a post-processing filter that can help to separate targets.
    ///
    /// # Arguments
    ///
    /// * `percent` - The sharpener value, from 0 (disabled) to 99.
    pub fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_sharpener_percent(self.config.as_mut(), percent)
        };
        Self::check_status(status)
    }

    /// Gets the current target order.
    pub fn get_target_order(&mut self) -> Result<u8, DriverError> {
        let mut target_order: u8 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_target_order(self.config.as_mut(), &mut target_order)
        };
        Self::check_status(status).map(|_| target_order)
    }

    /// Sets the target order for ranging results.
    ///
    /// Use `VL53L5CX_TARGET_ORDER_CLOSEST` or `VL53L5CX_TARGET_ORDER_STRONGEST`.
    pub fn set_target_order(&mut self, target_order: TargetOrder) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_set_target_order(self.config.as_mut(), target_order as u8) };
        Self::check_status(status)
    }

    /// Stops the ranging session.
    pub fn stop_ranging(&mut self) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_stop_ranging(self.config.as_mut()) };
        Self::check_status(status)
    }

    /// Sets the sensor power mode.
    ///
    /// The available modes are `VL53L5CX_POWER_MODE_SLEEP` and `VL53L5CX_POWER_MODE_WAKEUP`.
    /// Use `VL53L5CX_POWER_MODE_SLEEP` for low-power consumption. The sensor can be woken
    /// up by setting the mode back to `VL53L5CX_POWER_MODE_WAKEUP`.
    pub fn set_power_mode(&mut self, power_mode: PowerMode) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_power_mode(self.config.as_mut(), power_mode as u8)
        };
        Self::check_status(status)
    }

    /// Gets the current sensor power mode.
    ///
    /// Returns either `VL53L5CX_POWER_MODE_SLEEP` or `VL53L5CX_POWER_MODE_WAKEUP`.
    pub fn get_power_mode(&mut self) -> Result<u8, DriverError> {
        let mut power_mode: u8 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_power_mode(self.config.as_mut(), &mut power_mode)
        };
        Self::check_status(status).map(|_| power_mode)
    }

    /// Sets the integration time in milliseconds.
    ///
    /// This function allows manually setting the sensor's exposure time.
    /// Increasing integration time can improve accuracy, especially at long range,
    /// but it reduces the maximum possible ranging frequency.
    pub fn set_integration_time_ms(&mut self, integration_time_ms: u32) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_integration_time_ms(self.config.as_mut(), integration_time_ms)
        };
        Self::check_status(status)
    }

    /// Gets the integration time in milliseconds.
    pub fn get_integration_time_ms(&mut self) -> Result<u32, DriverError> {
        let mut integration_time_ms: u32 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_integration_time_ms(
                self.config.as_mut(), &mut integration_time_ms
            )
        };
        Self::check_status(status).map(|_| integration_time_ms)
    }

    /// Gets the current ranging mode.
    ///
    /// Returns `VL53L5CX_RANGING_MODE_CONTINUOUS` or `VL53L5CX_RANGING_MODE_AUTONOMOUS`.
    pub fn get_ranging_mode(&mut self) -> Result<u8, DriverError> {
        let mut ranging_mode: u8 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_ranging_mode(self.config.as_mut(), &mut ranging_mode)
        };
        Self::check_status(status).map(|_| ranging_mode)
    }

    /// Sets the ranging mode.
    ///
    /// Use `VL53L5CX_RANGING_MODE_CONTINUOUS` or `VL53L5CX_RANGING_MODE_AUTONOMOUS`.
    /// Autonomous mode is required for setting a custom integration time.
    pub fn set_ranging_mode(&mut self, ranging_mode: RangingMode) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_ranging_mode(self.config.as_mut(), ranging_mode as u8)
        };
        Self::check_status(status)
    }

    /// Gets the VHV (VCSEL High Voltage) repeat count.
    ///
    /// This value determines the number of frames between periodic temperature
    /// compensations. A value of 0 disables this feature.
    pub fn get_vhv_repeat_count(&mut self) -> Result<u32, DriverError> {
        let mut repeat_count: u32 = 0;
        let status = unsafe {
            bindings::vl53l5cx_get_VHV_repeat_count(self.config.as_mut(), &mut repeat_count)
        };
        Self::check_status(status).map(|_| repeat_count)
    }

    /// Sets the VHV (VCSEL High Voltage) repeat count.
    ///
    /// Setting a repeat count other than 0 enables periodic temperature
    /// calibration every N frames.
    ///
    /// # Arguments
    ///
    /// * `repeat_count` - Number of frames between temperature compensations.
    ///   Set to 0 to disable (default).
    pub fn set_vhv_repeat_count(&mut self, repeat_count: u32) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_VHV_repeat_count(self.config.as_mut(), repeat_count)
        };
        Self::check_status(status)
    }

    /// Enables the internal VCSEL charge pump.
    ///
    /// This is the default state.
    pub fn enable_internal_cp(&mut self) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_enable_internal_cp(self.config.as_mut()) };
        Self::check_status(status)
    }

    /// Disables the internal VCSEL charge pump to optimize power consumption.
    /// This should only be used if AVDD is 3.3V.
    pub fn disable_internal_cp(&mut self) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_disable_internal_cp(self.config.as_mut()) };
        Self::check_status(status)
    }

    /// Starts a ranging session.
    pub fn start_ranging(&mut self) -> Result<(), DriverError> {
        let status = unsafe { bindings::vl53l5cx_start_ranging(self.config.as_mut()) };
        Self::check_status(status)
    }

    /// Checks if new ranging data is ready to be read.
    pub fn check_data_ready(&mut self) -> Result<bool, DriverError> {
        let mut is_ready: u8 = 0;
        let status = unsafe { bindings::vl53l5cx_check_data_ready(self.config.as_mut(), &mut is_ready) };
        Self::check_status(status).map(|_| is_ready != 0)
    }

    /// Gets the latest ranging data from the sensor.
    pub fn get_ranging_data(&mut self) -> Result<bindings::VL53L5CX_ResultsData, DriverError> {
        let mut results: bindings::VL53L5CX_ResultsData = unsafe { core::mem::zeroed() };
        let status = unsafe { bindings::vl53l5cx_get_ranging_data(self.config.as_mut(), &mut results) };
        Self::check_status(status).map(|_| results)
    }

    /// Calibrates the sensor for crosstalk (Xtalk) from a cover glass.
    ///
    /// This function should be used when a protective cover is placed over the sensor.
    /// It calibrates the sensor to ignore reflections from the cover.
    /// The sensor must be ranging to perform the calibration.
    ///
    /// # Arguments
    ///
    /// * `reflectance_percent` - The target reflectance in percent (1-99). Recommended: 3%.
    /// * `distance_mm` - The target distance in millimeters. Recommended: 600mm.
    /// * `nb_samples` - The number of samples to use for calibration. Recommended: 16.
    #[cfg(feature = "xtalk")]
    pub fn calibrate_xtalk(&mut self, reflectance_percent: u16, distance_mm: u16, nb_samples: u8) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_calibrate_xtalk(
                self.config.as_mut(),
                reflectance_percent,
                nb_samples,
                distance_mm,
            )
        };
        Self::check_status(status)
    }

    /// Retrieves the crosstalk calibration data from the sensor.
    ///
    /// This data can be saved to non-volatile memory and re-applied on startup
    /// using `set_caldata_xtalk` to avoid re-running calibration on every power-on.
    /// The returned buffer will have a size of `VL53L5CX_XTALK_BUFFER_SIZE`.
    #[cfg(feature = "xtalk")]
    pub fn get_caldata_xtalk(&mut self) -> Result<Vec<u8>, DriverError> {
        // The C driver expects a buffer of size VL53L5CX_XTALK_BUFFER_SIZE.
        let mut xtalk_data = Vec::with_capacity(VL53L5CX_XTALK_BUFFER_SIZE);
        unsafe {
            xtalk_data.set_len(VL53L5CX_XTALK_BUFFER_SIZE);
        }

        let status = unsafe {
            bindings::vl53l5cx_get_caldata_xtalk(self.config.as_mut(), xtalk_data.as_mut_ptr())
        };

        Self::check_status(status).map(|_| xtalk_data)
    }

    /// Sets the crosstalk calibration data on the sensor.
    ///
    /// This is used to restore a previously saved calibration profile without
    /// needing to re-run the calibration process. The provided data must have
    /// a size of `VL53L5CX_XTALK_BUFFER_SIZE`.
    #[cfg(feature = "xtalk")]
    pub fn set_caldata_xtalk(&mut self, xtalk_data: &[u8]) -> Result<(), DriverError> {
        // The C driver requires the buffer to be exactly VL53L5CX_XTALK_BUFFER_SIZE bytes.
        if xtalk_data.len() != VL53L5CX_XTALK_BUFFER_SIZE {
            // Using a sensor-specific error code might be appropriate here.
            return Err(DriverError::Sensor(SensorStatus::InvalidParam));
        }
        let status = unsafe { bindings::vl53l5cx_set_caldata_xtalk(self.config.as_mut(), xtalk_data.as_ptr() as *mut u8) };
        Self::check_status(status)
    }

    /// Sets detection thresholds for interrupt generation.
    ///
    /// This powerful feature allows the sensor to trigger an interrupt only when an
    /// object meets specific criteria (e.g., closer than a certain distance).
    /// This is ideal for low-power presence detection, as the host processor
    /// doesn't need to continuously poll and parse ranging data.
    ///
    /// # Arguments
    ///
    /// * `thresholds` - A mutable reference to a `VL53L5CX_DetectionThresholds` struct.
    ///   The user must create and populate this struct, which can define up to two
    ///   independent thresholds.
    #[cfg(feature = "thresholds")]
    pub fn set_detection_thresholds(
        &mut self,
        thresholds: &mut bindings::VL53L5CX_DetectionThresholds,
    ) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_detection_thresholds(self.config.as_mut(), thresholds)
        };
        Self::check_status(status)
    }

    /// Initializes the motion indicator feature.
    ///
    /// The motion indicator is a plugin that detects motion within the sensor's FoV.
    /// This function must be called before using the motion indicator.
    /// The `VL53L5CX_ResultsData` struct contains a `motion_indicator` field with
    /// motion data per zone.
    ///
    /// # Arguments
    ///
    /// * `motion_config` - A mutable reference to a `VL53L5CX_Motion_Configuration` struct.
    /// * `resolution` - The sensor resolution (`VL53L5CX_RESOLUTION_4X4` or `VL53L5CX_RESOLUTION_8X8`).
    #[cfg(motion_or_thresholds)]
    pub fn motion_indicator_init(
        &mut self,
        motion_config: &mut bindings::VL53L5CX_Motion_Configuration,
        resolution: u8,
    ) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_motion_indicator_init(self.config.as_mut(), motion_config, resolution)
        };
        Self::check_status(status)
    }

    /// Configures the distance window for the motion indicator.
    ///
    /// # Arguments
    ///
    /// * `motion_config` - A mutable reference to a `VL53L5CX_Motion_Configuration` struct.
    /// * `distance_min_mm` - The minimum distance for motion detection (400mm to 4000mm).
    /// * `distance_max_mm` - The maximum distance for motion detection (400mm to 4000mm).
    ///   The window (max - min) must be less than 1500mm.
    #[cfg(motion_or_thresholds)]
    pub fn motion_indicator_set_distance_motion(
        &mut self,
        motion_config: &mut bindings::VL53L5CX_Motion_Configuration,
        distance_min_mm: u16,
        distance_max_mm: u16,
    ) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_motion_indicator_set_distance_motion(
                self.config.as_mut(), motion_config, distance_min_mm, distance_max_mm
            )
        };
        Self::check_status(status)
    }

    /// Private helper function to check status codes from the C driver.
    fn check_status(status: u8) -> Result<(), DriverError> {
        if status == 255 {
            // This is our conventional error code for I2C failures in `platform.rs`
            Err(DriverError::I2c)
        } else if status == 0 {
            Ok(())
        } else {
            // Any other non-zero status is a sensor-specific error
            Err(DriverError::Sensor(status.into()))
        }
    }
}

// We must implement Drop to properly clean up the `Box<dyn I2c>` that we
// converted into a raw pointer. Failure to do so would result in a memory leak.
impl Drop for Vl53l5cx {
    fn drop(&mut self) {
        // Best-effort attempt to stop the sensor to put it in a known state.
        // We ignore the result because we can't do anything about an error during drop.
        let _ = self.stop_ranging();

        // Reconstruct the Box from the raw pointer and let it be dropped,
        // which deallocates the memory for our PlatformResources struct.
        unsafe {
            let _ = Box::from_raw(self.platform_resources_ptr);
        }
    }
}

// The C struct `VL53L5CX_Configuration` does not derive Default.
// We need to implement it manually.
impl Default for bindings::VL53L5CX_Configuration {
    fn default() -> Self {
        // Safe way to create a zero-initialized struct.
        unsafe { core::mem::zeroed() }
    }
}
