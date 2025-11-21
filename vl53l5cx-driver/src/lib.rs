#![cfg_attr(not(feature = "std"), no_std)]
//! A platform-agnostic Rust driver for the STMicroelectronics VL53L5CX Time-of-Flight (ToF) 8x8 multizone ranging sensor.
//!
//! This crate provides a safe, idiomatic Rust interface to the official ST Ultra Lite Driver (ULD) C-API.
//!
//! # Firmware Upload
//! The VL53L5CX sensor does not have persistent on-chip memory for its firmware. It requires a firmware blob (~90KB)
//! to be loaded into its RAM via I2C during initialization. This driver handles the firmware upload automatically
//! when you call the [`Vl53l5cx::new`] constructor.
//!
//! # More Information and Datasheet
//! For complete details on the sensor's capabilities, please refer to the STMicroelectronics VL53L5CX Datasheet.

#[cfg(not(feature = "std"))]
extern crate alloc;

use embedded_hal::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};

// Use the external sys-crate for C bindings.
use vl53l5cx_sys::{self as bindings, wrappers};

// Re-export the most commonly used structs.
pub use bindings::{VL53L5CX_DetectionThresholds, VL53L5CX_Motion_Configuration, VL53L5CX_ResultsData};

// Include the platform glue module.
pub mod platform;

/// The size of the Xtalk calibration data buffer.
#[cfg(feature = "xtalk")]
pub const VL53L5CX_XTALK_BUFFER_SIZE: usize = 776;

// --- TRAIT DEFINITION ---
/// A trait to abstract away the generic `I` and `D` types for the C callback.
pub trait PlatformTrait<'a> {
    fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), platform::PlatformError>;
    fn write_read(&mut self, addr: u8, wr_buf: &[u8], rd_buf: &mut [u8]) -> Result<(), platform::PlatformError>;
    fn delay_ms(&mut self, ms: u32);
}

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

#[derive(Debug, Copy, Clone)]
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
    I2c,
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

/// A newtype wrapper to ensure the sharpener percentage is within the valid range (0-99).
#[derive(Debug, Copy, Clone)]
pub struct SharpenerPercent(u8);

impl SharpenerPercent {
    pub fn new(percent: u8) -> Option<Self> {
        if percent <= 99 { Some(Self(percent)) } else { None }
    }
    pub fn value(&self) -> u8 { self.0 }
}

/// Represents a VL53L5CX sensor instance.
///
/// # Memory Safety
/// **Important:** This struct contains self-referential pointers required for the underlying C driver's callback mechanism.
/// It **MUST NOT** be moved in memory after it is created. If you need to store it in another struct,
/// you must use a stable pointer like `Box::pin`.
pub struct Vl53l5cx<'a, I, D>
where
    I: I2c<SevenBitAddress> + 'a,
    D: DelayNs + 'a,
{
    /// The ST driver configuration struct.
    config: bindings::VL53L5CX_Configuration,
    /// The I2C bus.
    pub i2c: I,
    /// The delay provider.
    pub delay: D,
    
    /// Internal storage for the Trait Object pointer (Fat Pointer).
    // FIX: Added `+ 'a` to explicitly allow non-static lifetimes
    proxy: *mut (dyn PlatformTrait<'a> + 'a),
}

impl<'a, I, D> Vl53l5cx<'a, I, D>
where
    I: I2c<SevenBitAddress> + 'a,
    D: DelayNs + 'a,
{
    /// Creates a new driver instance, initializes the sensor, and uploads the firmware.
    ///
    /// This function performs the complete sensor initialization, which includes a lengthy I2C transaction
    /// to load the ~90KB firmware into the sensor's RAM.
    ///
    /// # Warning
    /// Due to internal pointers used for C callbacks, the returned `Vl53l5cx` struct **MUST NOT** be moved in memory.
    pub fn new(i2c: I, delay: D, address: u8) -> Result<Self, DriverError> {
        // FIX: Updated cast to include `+ 'a`
        let null_fat_ptr = core::ptr::null_mut::<Self>() as *mut (dyn PlatformTrait<'a> + 'a);

        let mut sensor = Self {
            config: unsafe { core::mem::zeroed() },
            i2c,
            delay,
            proxy: null_fat_ptr, 
        };

        // 1. Create the Fat Pointer to `self` (as a Trait Object)
        // FIX: Updated type annotation to include `+ 'a`
        let proxy_ptr: *mut (dyn PlatformTrait<'a> + 'a) = &mut sensor;
        
        // 2. Store this Fat Pointer inside the struct itself
        sensor.proxy = proxy_ptr;

        // 3. Configure the C struct to point to our `proxy` field.
        let platform_ptr = &mut sensor.config.platform as *mut _ as *mut platform::VL53L5CX_Platform;
        unsafe {
            (*platform_ptr).p_com = &mut sensor.proxy as *mut _ as *mut core::ffi::c_void;
            (*platform_ptr).address = address as u16;
            
            // Call C init
            let status = wrappers::init(&mut sensor.config);
            Self::check_ok(status)?;
        }

        Ok(sensor)
    }

    /// Returns the version string of the underlying ST ULD C-driver.
    ///
    /// The version is retrieved from the `VL53L5CX_API_REVISION` C-macro defined
    /// in the vendor's source code.
    pub fn get_version(&self) -> Result<&'static str, DriverError> {
        // The `refresh_pointers` call is not strictly necessary here as we are not using the platform callbacks,
        // but it is good practice to include for consistency.
        // self.refresh_pointers();
        let c_str = unsafe {
            // SAFETY: The binding guarantees that VL53L5CX_API_REVISION points to a valid, null-terminated C string.
            core::ffi::CStr::from_bytes_with_nul_unchecked(bindings::VL53L5CX_API_REVISION)
        };
        // Note: The conversion from CStr to &str is Fallible, but the ST version string is ASCII.
        let version_str = c_str.to_str().unwrap_or("Unknown Version");
        Ok(version_str)
    }

    /// Updates the C driver's internal pointers to point to the current memory location of this struct.
    /// This must be called before any C function if the struct has moved.
    fn refresh_pointers(&mut self) {
        unsafe {
            // 1. Point proxy to the current address of self
            let proxy_ptr: *mut (dyn PlatformTrait<'a> + 'a) = self;
            self.proxy = proxy_ptr;

            // 2. Point C platform.p_com to the current address of self.proxy
            let platform_ptr = &mut self.config.platform as *mut bindings::VL53L5CX_Platform;
            (*platform_ptr).p_com = &mut self.proxy as *mut _ as *mut core::ffi::c_void;
        }
    }

    /// Checks if the sensor is alive and responding.
    ///
    /// This function reads the sensor's device and revision IDs. A successful read
    /// returning the expected ID (`0xF0` for device, `0x02` for revision) indicates
    /// that the sensor is powered and communicating correctly.
    pub fn is_alive(&mut self) -> Result<bool, DriverError> {
        self.refresh_pointers();
        let (status, is_alive) = wrappers::is_alive(&mut self.config);
        Self::check_ok(status).map(|_| is_alive != 0)
    }

    /// Sets a new I2C address for the sensor.
    ///
    /// # Note
    /// The new address is temporary and will be reset to the default (`0x29`)
    /// when the sensor is power-cycled.
    pub fn set_i2c_address(&mut self, address: u8) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_i2c_address(&mut self.config, address as u16);
        Self::check_ok(status)
    }

    /// Sets the sensor's ranging resolution.
    ///
    /// This setting impacts the maximum possible ranging frequency:
    /// - `Resolution::Res4x4` (16 zones): Max 60Hz
    /// - `Resolution::Res8x8` (64 zones): Max 15Hz
    pub fn set_resolution(&mut self, resolution: Resolution) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_resolution(&mut self.config, resolution as u8);
        Self::check_ok(status)
    }

    /// Sets the ranging frequency in Hz.
    ///
    /// The maximum frequency depends on the configured resolution:
    /// - `Resolution::Res4x4`: 1Hz to 60Hz
    /// - `Resolution::Res8x8`: 1Hz to 15Hz
    pub fn set_ranging_frequency_hz(&mut self, frequency: u8) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_ranging_frequency_hz(&mut self.config, frequency);
        Self::check_ok(status)
    }

    /// Gets the current sharpener percentage.
    pub fn get_sharpener_percent(&mut self) -> Result<u8, DriverError> {
        self.refresh_pointers();
        let (status, sharpener_percent) = wrappers::get_sharpener_percent(&mut self.config);
        Self::check_ok(status).map(|_| sharpener_percent)
    }

    /// Sets the sharpener percentage.
    ///
    /// The sharpener is an edge-detection filter that can help distinguish targets.
    /// The valid range is 0% (disabled) to 99%.
    pub fn set_sharpener_percent(&mut self, percent: SharpenerPercent) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_sharpener_percent(&mut self.config, percent.value());
        Self::check_ok(status)
    }

    /// Gets the current target order.
    /// The target order determines which target is reported when multiple are detected in a zone.
    pub fn get_target_order(&mut self) -> Result<TargetOrder, DriverError> {
        self.refresh_pointers();
        let (status, target_order) = wrappers::get_target_order(&mut self.config);
        Self::check_ok(status).map(|_| match target_order {
            1 => TargetOrder::Closest,
            2 => TargetOrder::Strongest,
            _ => TargetOrder::Closest,
        })
    }

    /// Sets the target order.
    /// By default, the sensor is configured with `TargetOrder::Strongest`.
    pub fn set_target_order(&mut self, target_order: TargetOrder) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_target_order(&mut self.config, target_order as u8);
        Self::check_ok(status)
    }

    /// Stops the current ranging session.
    /// This must be used when the sensor is streaming, after calling `start_ranging()`.
    pub fn stop_ranging(&mut self) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::stop_ranging(&mut self.config);
        Self::check_ok(status)
    }

    /// Sets the sensor's power mode.
    ///
    /// - `PowerMode::Sleep`: Puts the sensor in a low-power state, retaining firmware and configuration.
    /// - `PowerMode::Wakeup`: Wakes the sensor from sleep mode.
    /// Ensure the device is not streaming before calling this function.
    pub fn set_power_mode(&mut self, power_mode: PowerMode) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_power_mode(&mut self.config, power_mode as u8);
        Self::check_ok(status)
    }

    /// Gets the current power mode of the sensor.
    pub fn get_power_mode(&mut self) -> Result<PowerMode, DriverError> {
        self.refresh_pointers();
        let (status, power_mode) = wrappers::get_power_mode(&mut self.config);
        Self::check_ok(status).map(|_| match power_mode {
            0 => PowerMode::Sleep,
            _ => PowerMode::Wakeup,
        })
    }

    /// Sets the integration time in milliseconds.
    ///
    /// This function allows manual control over the sensor's exposure time.
    /// The valid range is from 2ms to 1000ms.
    pub fn set_integration_time_ms(&mut self, integration_time_ms: u32) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_integration_time_ms(&mut self.config, integration_time_ms);
        Self::check_ok(status)
    }

    /// Gets the current integration time in milliseconds.
    pub fn get_integration_time_ms(&mut self) -> Result<u32, DriverError> {
        self.refresh_pointers();
        let (status, integration_time_ms) = wrappers::get_integration_time_ms(&mut self.config);
        Self::check_ok(status).map(|_| integration_time_ms)
    }

    /// Gets the current ranging mode.
    /// The default mode is `Autonomous`.
    pub fn get_ranging_mode(&mut self) -> Result<RangingMode, DriverError> {
        self.refresh_pointers();
        let (status, ranging_mode) = wrappers::get_ranging_mode(&mut self.config);
        Self::check_ok(status).map(|_| match ranging_mode {
            1 => RangingMode::Continuous,
            3 => RangingMode::Autonomous,
            _ => RangingMode::Continuous,
        })
    }

    /// Sets the ranging mode.
    ///
    /// - `RangingMode::Continuous`: The sensor continuously acquires frames at the maximum possible speed.
    /// - `RangingMode::Autonomous`: The sensor acquires frames based on the configured ranging frequency.
    ///   This mode allows for a precise integration time to be set.
    pub fn set_ranging_mode(&mut self, ranging_mode: RangingMode) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_ranging_mode(&mut self.config, ranging_mode as u8);
        Self::check_ok(status)
    }

    /// Gets the number of frames between automatic temperature compensations (VHV calibration).
    pub fn get_vhv_repeat_count(&mut self) -> Result<u32, DriverError> {
        self.refresh_pointers();
        let (status, repeat_count) = wrappers::get_vhv_repeat_count(&mut self.config);
        Self::check_ok(status).map(|_| repeat_count)
    }

    /// Sets the number of frames between automatic temperature compensations (VHV calibration).
    ///
    /// Setting a `repeat_count` other than 0 will cause the firmware to automatically run a
    /// temperature calibration every `N` frames.
    ///
    /// Setting to `0` disables this feature (default).
    pub fn set_vhv_repeat_count(&mut self, repeat_count: u32) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_vhv_repeat_count(&mut self.config, repeat_count);
        Self::check_ok(status)
    }

    /// Enables the internal VCSEL charge pump.
    /// This is enabled by default.
    pub fn enable_internal_cp(&mut self) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::enable_internal_cp(&mut self.config);
        Self::check_ok(status)
    }

    /// Disables the internal VCSEL charge pump to optimize power consumption.
    ///
    /// # Warning
    /// This function should only be used if the analog supply voltage (AVDD) is 3.3V.
    pub fn disable_internal_cp(&mut self) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::disable_internal_cp(&mut self.config);
        Self::check_ok(status)
    }

    /// Starts a ranging session. The ranging mode must be configured before this call.
    ///
    /// This function locks the sensor's configuration and begins the measurement process.
    /// After calling this, you can poll for new data using `check_data_ready()`.
    pub fn start_ranging(&mut self) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::start_ranging(&mut self.config);
        Self::check_ok(status)
    }

    /// Checks if a new set of ranging data is ready to be read from the sensor.
    ///
    /// This function is typically used in a polling loop to wait for new measurements.
    ///
    /// ```rust,no_run
    /// # use vl53l5cx_driver::{Vl53l5cx, DriverError};
    /// # fn poll(sensor: &mut Vl53l5cx<impl embedded_hal::i2c::I2c, impl embedded_hal::delay::DelayNs>) -> Result<(), DriverError> {
    /// while !sensor.check_data_ready()? {
    ///     // Wait or yield
    /// }
    /// let results = sensor.get_ranging_data()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn check_data_ready(&mut self) -> Result<bool, DriverError> {
        self.refresh_pointers();
        let (status, is_ready) = wrappers::check_data_ready(&mut self.config);
        Self::check_ok(status).map(|_| is_ready != 0)
    }

    /// Gets the latest ranging data from the sensor.
    ///
    /// This function retrieves a `VL53L5CX_ResultsData` struct containing the measurement
    /// results for all zones, which includes:
    /// - `distance_mm`: Distance to the target in millimeters.
    /// - `signal_per_spad`: Signal rate of the reflected light.
    /// - `ambient_per_spad`: Ambient light noise rate.
    /// - And other diagnostic data.
    pub fn get_ranging_data(&mut self) -> Result<bindings::VL53L5CX_ResultsData, DriverError> {
        self.refresh_pointers();
        let (status, results) = wrappers::get_ranging_data(&mut self.config);
        Self::check_ok(status).map(|_| results)
    }

    #[cfg(feature = "xtalk")]
    /// Performs a crosstalk (Xtalk) calibration.
    /// This should be done if the sensor is placed behind a protective cover glass.
    /// The target must be placed at a distance between 600mm and 3000mm.
    pub fn calibrate_xtalk(&mut self, reflectance_percent: u16, distance_mm: u16, nb_samples: u8) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::calibrate_xtalk(&mut self.config, reflectance_percent, nb_samples, distance_mm);
        Self::check_ok(status)
    }

    #[cfg(feature = "xtalk")]
    /// Retrieves the crosstalk calibration data from the sensor's RAM.
    /// The returned `alloc::vec::Vec<u8>` can be stored in non-volatile memory for later use with `set_caldata_xtalk`.
    pub fn get_caldata_xtalk(&mut self) -> Result<alloc::vec::Vec<u8>, DriverError> {
        self.refresh_pointers();
        let mut xtalk_data = alloc::vec::Vec::with_capacity(VL53L5CX_XTALK_BUFFER_SIZE);
        unsafe { xtalk_data.set_len(VL53L5CX_XTALK_BUFFER_SIZE); }
        let status = wrappers::get_caldata_xtalk(&mut self.config, xtalk_data.as_mut_ptr());
        Self::check_ok(status).map(|_| xtalk_data)
    }

    #[cfg(feature = "xtalk")]
    /// Writes a previously saved crosstalk calibration buffer to the sensor.
    /// This avoids the need to run a new calibration on every power-up.
    pub fn set_caldata_xtalk(&mut self, xtalk_data: &[u8]) -> Result<(), DriverError> {
        self.refresh_pointers();
        if xtalk_data.len() != VL53L5CX_XTALK_BUFFER_SIZE { return Err(DriverError::Sensor(SensorStatus::InvalidParam)); }
        let status = wrappers::set_caldata_xtalk(&mut self.config, xtalk_data.as_ptr() as *mut u8);
        Self::check_ok(status)
    }

    #[cfg(feature = "thresholds")]
    /// Programs the detection thresholds.
    ///
    /// This function takes an array of up to 64 `VL53L5CX_DetectionThresholds` checkers.
    /// Each checker defines a condition (e.g., distance < 500mm in zone 5) that can be used
    /// to trigger an interrupt.
    pub fn set_detection_thresholds(&mut self, thresholds: &mut bindings::VL53L5CX_DetectionThresholds) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::set_detection_thresholds(&mut self.config, thresholds);
        Self::check_ok(status)
    }

    #[cfg(any(feature = "motion", feature = "thresholds"))]
    /// Initializes the motion indicator functionality.
    /// By default, the indicator is programmed to monitor movements between 400mm and 1500mm.
    /// This function must be called before using other motion indicator methods.
    pub fn motion_indicator_init(&mut self, motion_config: &mut bindings::VL53L5CX_Motion_Configuration, resolution: Resolution) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::motion_indicator_init(&mut self.config, motion_config, resolution as u8);
        Self::check_ok(status)
    }

    #[cfg(any(feature = "motion", feature = "thresholds"))]
    /// Sets the minimum and maximum distance for motion detection.
    ///
    /// This function changes the working distance of the motion indicator.
    /// - `distance_min_mm`: Minimum distance for indicator (min 400mm, max 4000mm).
    /// - `distance_max_mm`: Maximum distance for indicator (min 400mm, max 4000mm).
    pub fn motion_indicator_set_distance_motion(&mut self, motion_config: &mut bindings::VL53L5CX_Motion_Configuration, distance_min_mm: u16, distance_max_mm: u16) -> Result<(), DriverError> {
        self.refresh_pointers();
        let status = wrappers::motion_indicator_set_distance_motion(&mut self.config, motion_config, distance_min_mm, distance_max_mm);
        Self::check_ok(status)
    }

    fn check_ok(status: u8) -> Result<(), DriverError> {
        if status == 255 { Err(DriverError::I2c) }
        else if status == 0 { Ok(()) }
        else { Err(DriverError::Sensor(status.into())) }
    }
}

impl<'a, I: I2c<SevenBitAddress>, D: DelayNs> Drop for Vl53l5cx<'a, I, D> {
    fn drop(&mut self) {
        let _ = self.stop_ranging();
    }
}