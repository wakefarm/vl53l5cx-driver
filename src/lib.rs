#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![cfg_attr(not(feature = "std"), no_std)]
#![allow(dead_code)]

use embedded_hal::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};
use core::ffi::c_void;

#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(not(feature = "std"))]
use alloc::boxed::Box;

// Include the bindgen-generated bindings.
// We wrap the `include!` in a module to give the generated code its own namespace.
mod bindings {
    // This file is created in the OUT_DIR directory during the build process.
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
// Include the platform glue module.
pub mod platform;

#[derive(Debug)]
pub enum DriverError {
    /// An I2C communication error occurred in the platform layer.
    /// The C driver reports this as status 255.
    I2c,
    /// The sensor returned a non-zero status code, indicating an error.
    Sensor(u8),
}

impl core::fmt::Display for DriverError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DriverError::I2c => write!(f, "I2C communication error"),
            DriverError::Sensor(status) => write!(f, "Sensor driver error: status code {}", status),
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
            // Now we can safely cast the thin pointer to void*
            (*platform_ptr).p_com = platform_resources_ptr as *mut c_void;

            // 2. CRITICAL: Store the address so the C-driver uses it!
            // embedded-hal expects 7-bit.
            (*platform_ptr).address = address as u16;
        }

        // 3. Call the C API to initialize the sensor.
        //    This is `unsafe` because we are calling a C function and passing
        //    a raw pointer to our config struct.
        let status = unsafe { bindings::vl53l5cx_init(config.as_mut()) };

        // 4. Check the status code and return a proper Rust `Result`.
        Self::check_status(status).map(|_| Self {
            config,
            platform_resources_ptr, // Store the ptr for Drop
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

    /// Sets the sensor resolution.
    ///
    /// The resolution can be either 4x4 (`VL53L5CX_RESOLUTION_4X4`) or
    /// 8x8 (`VL53L5CX_RESOLUTION_8X8`).
    pub fn set_resolution(&mut self, resolution: u8) -> Result<(), DriverError> {
        let status = unsafe {
            bindings::vl53l5cx_set_resolution(self.config.as_mut(), resolution)
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

    /// Private helper function to check status codes from the C driver.
    fn check_status(status: u8) -> Result<(), DriverError> {
        if status == 255 {
            // This is our conventional error code for I2C failures in `platform.rs`
            Err(DriverError::I2c)
        } else if status == 0 {
            Ok(())
        } else {
            // Any other non-zero status is a sensor-specific error
            Err(DriverError::Sensor(status))
        }
    }
}

// We must implement Drop to properly clean up the `Box<dyn I2c>` that we
// converted into a raw pointer. Failure to do so would result in a memory leak.
impl Drop for Vl53l5cx {
    fn drop(&mut self) {
        // Reconstruct the Box from the raw pointer and let it be dropped,
        // which deallocates the memory for our PlatformResources struct.
        unsafe {
            // Convert the thin pointer back into the outer box
            // This will drop the outer box, which in turn drops the inner box (the actual I2C bus)
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
