#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(dead_code)]

use embedded_hal::i2c::{I2c, SevenBitAddress};
use thiserror::Error;

// Include the bindgen-generated bindings.
// We wrap the `include!` in a module to give the generated code its own namespace.
mod bindings {
    // This file is created in the OUT_DIR directory during the build process.
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
// Include the platform glue module.
pub mod platform;

/// A comprehensive error type for the VL53L5CX driver.
#[derive(Error, Debug)]
pub enum DriverError {
    /// An I2C communication error occurred in the platform layer.
    /// The C driver reports this as status 255.
    #[error("I2C communication error")]
    I2c,
    /// The sensor returned a non-zero status code, indicating an error.
    #[error("Sensor driver error: status code {0}")]
    Sensor(u8),
}

/// Represents a VL53L5CX sensor instance.
pub struct Vl53l5cx {
    /// The ST driver configuration struct. This is the main handle used by the C API.
    /// It's boxed to give it a stable memory address.
    config: Box<bindings::VL53L5CX_Configuration>,
}

impl Vl53l5cx {
    /// Creates a new driver instance and initializes the sensor.
    ///
    /// This function takes ownership of the I2C bus, initializes the platform.
    /// layer, and then calls the C driver's `vl53l5cx_init` function.
    pub fn new(
        i2c_bus: Box<dyn I2c<SevenBitAddress, Error = platform::PlatformError> + Send>,
    ) -> Result<Self, DriverError> {
        // 1. Initialize our platform by giving it the I2C bus.
        //    This must be done *before* any C driver functions are called.
        platform::set_i2c_bus(i2c_bus);

        // 2. Create the C configuration struct on the heap.
        let mut config: Box<bindings::VL53L5CX_Configuration> = Box::new(unsafe { core::mem::zeroed() });

        // 3. Call the C API to initialize the sensor.
        //    This is `unsafe` because we are calling a C function and
        //    passing a raw pointer to our config struct.
        let status = unsafe { bindings::vl53l5cx_init(config.as_mut()) };

        // 4. Check the status code and return a proper Rust `Result`. Map
        //    the platform-specific I2C error code to our new error variant.
        Self::check_status(status).map(|_| Self { config })
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
        } else if status == bindings::VL53L5CX_STATUS_OK as u8 {
            Ok(())
        } else {
            // Any other non-zero status is a sensor-specific error
            Err(DriverError::Sensor(status))
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

// When our Vl53l5cx struct is dropped, we should ensure the C driver's
// platform termination function is called to clean up.
impl Drop for Vl53l5cx {
    fn drop(&mut self) {
        // Release the I2C bus from the global static context in `platform.rs`.
        // This is good RAII practice.
        platform::terminate_platform();
    }
}
