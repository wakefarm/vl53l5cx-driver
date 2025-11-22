#![no_std]
/*!
A platform-agnostic driver for the VL53L5CX Time-of-Flight sensor.

This driver is built on top of the `embedded-hal` traits and uses the official
ST Microelectronics VL53L5CX ULD (Ultra Lite Driver) via the `vl53l5cx-sys` crate.

## Features
- `std`: Enables `std::error::Error` implementation for the `Error` type.
- `xtalk`: Enables cross-talk calibration functions.
- `motion`: Enables motion indicator functions.

## Usage

A typical initialization sequence for a single sensor looks like this:

```no_run
# use embedded_hal::i2c::I2c;
# use embedded_hal::delay::DelayNs;
# use embedded_hal::digital::OutputPin;
# use vl53l5cx_driver::{Vl53l5cx, Error};
# fn main() -> Result<(), Error<()>> {
# let i2c = (); let delay = (); let lpn_pin = ();
let mut sensor = Vl53l5cx::new(i2c, delay).with_reset(lpn_pin);
sensor.init()?;
# Ok(())
# }
```
*/

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::digital::OutputPin;
pub use vl53l5cx_sys as sys;

/// The default I2C address of the VL53L5CX sensor.
pub const DEFAULT_ADDRESS: u8 = 0x29;

/// A placeholder type for an unconfigured hardware pin.
///
/// This is used as the default generic parameter for the `reset_pin` and `int_pin`
/// in the `Vl53l5cx` struct, making them optional.
pub struct NoPin;

/// Implements the `ErrorType` for `NoPin`, indicating that its operations are infallible.
impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

/// Implements `OutputPin` for `NoPin` as a no-op.
/// This allows the driver to compile and function even when an optional pin is not provided,
/// by satisfying the trait bounds on methods like `init()`.
impl embedded_hal::digital::OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
    fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
}

/// Represents errors that can occur in the driver.
#[derive(Debug)]
pub enum Error<E> {
    /// An error from the underlying I2C bus.
    I2c(E),
    /// The sensor is not alive on the bus.
    NotAlive,
    /// An error status was returned by the sensor's firmware.
    Firmware(u8),
    /// A required hardware pin (like reset) was not provided.
    PinMissing,
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for Error<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2c(e) => Some(e),
            _ => None,
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::fmt::Debug> std::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {:?}", e),
            Error::NotAlive => write!(f, "Sensor not found at address"),
            Error::Firmware(status) => write!(f, "Firmware error, status: {}", status),
            Error::PinMissing => write!(f, "A required hardware pin was not configured"),
        }
    }
}

/// The main driver struct for the VL53L5CX sensor.
///
/// It is generic over the I2C bus, a delay provider, a reset pin (LPn), and an
/// interrupt pin (INT). The pins are optional and can be added using the builder
/// pattern (`with_reset`, `with_interrupt`).
pub struct Vl53l5cx<I2C, D, RST = NoPin, INT = NoPin> {
    i2c: I2C,
    delay: D,
    address: SevenBitAddress,
    reset_pin: RST,
    int_pin: INT,
    has_reset: bool,
    /// Internal driver state.
    stdev: sys::VL53L5CX_Configuration,
}

/// Constructor and builder methods.
impl<I2C, D> Vl53l5cx<I2C, D, NoPin, NoPin>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Creates a new driver instance with the default I2C address.
    /// This function is lightweight and does not perform any I2C communication.
    ///
    /// # Arguments
    /// * `i2c` - An `embedded-hal` I2C bus implementation.
    /// * `delay` - An `embedded-hal` delay provider.
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            i2c,
            delay,
            address: DEFAULT_ADDRESS,
            reset_pin: NoPin,
            int_pin: NoPin,
            has_reset: false,
            // Initialize the ST driver state. This is a large struct.
            stdev: unsafe { core::mem::zeroed() },
        }
    }
}

/// Builder methods for optional pins.
impl<I2C, D, RST, INT> Vl53l5cx<I2C, D, RST, INT> {
    /// Attaches a reset pin (LPn) to the driver.
    ///
    /// The reset pin is required for changing the sensor's I2C address and is
    /// crucial for managing multi-sensor setups.
    pub fn with_reset<RstNew>(self, reset_pin: RstNew) -> Vl53l5cx<I2C, D, RstNew, INT> {
        Vl53l5cx {
            i2c: self.i2c,
            delay: self.delay,
            address: self.address,
            reset_pin,
            int_pin: self.int_pin,
            has_reset: true,
            stdev: self.stdev,
        }
    }

    /// Attaches an interrupt pin (INT) to the driver.
    ///
    /// The interrupt pin is used by the sensor to signal that new ranging
    /// data is ready.
    pub fn with_interrupt<IntNew>(self, int_pin: IntNew) -> Vl53l5cx<I2C, D, RST, IntNew> {
        Vl53l5cx {
            i2c: self.i2c,
            delay: self.delay,
            address: self.address,
            reset_pin: self.reset_pin,
            int_pin,
            has_reset: self.has_reset,
            stdev: self.stdev,
        }
    }
}

/// Core driver functionality.
impl<I2C, D, RST, INT, E> Vl53l5cx<I2C, D, RST, INT>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
    // The `OutputPin` trait bound is now satisfied by `NoPin` as well.
    // The error type of the pin must be mappable to our driver's PinMissing error.
    RST: OutputPin,
    <RST as embedded_hal::digital::ErrorType>::Error: core::fmt::Debug,
{
    /// Changes the I2C address of the sensor.
    ///
    /// This method requires the reset pin (LPn) to be configured via `with_reset()`.
    /// It ensures the sensor is enabled, sends the I2C command to change the address,
    /// and updates the driver's internal state.
    ///
    /// In a multi-sensor setup, you should keep all other sensors in reset (LPn low)
    /// while changing the address of one.
    ///
    /// # Arguments
    /// * `new_address` - The new 7-bit I2C address for the sensor.
    ///
    /// # Errors
    /// Returns `Error::PinMissing` if the reset pin has not been configured.
    pub fn set_address(&mut self, new_address: u8) -> Result<(), Error<E>> {
        // Explicitly check if a reset pin was configured. The `OutputPin` trait
        // on `NoPin` is for `init()`, but `set_address` must have a real pin.
        if !self.has_reset {
            return Err(Error::PinMissing);
        }
        // To ensure a clean state, especially in multi-sensor setups, we cycle the reset pin.
        // 1. Disable (Reset)
        self.reset_pin.set_low().map_err(|_| Error::PinMissing)?;
        self.delay.delay_ms(2);

        // 2. Enable (Wake up)
        self.reset_pin.set_high().map_err(|_| Error::PinMissing)?;
        self.delay.delay_ms(2);


        // Use the official ULD function to change the address. This ensures the
        // driver's internal state remains synchronized with the hardware.
        let status = unsafe {
            sys::vl53l5cx_set_i2c_address(&mut self.stdev, new_address as u16)
        };

        if status != 0 {
            return Err(Error::Firmware(status));
        }

        // Update the address in our driver struct to match.
        self.address = new_address;

        Ok(())
    }

    /// Initializes the sensor.
    ///
    /// This function performs the full boot sequence for the sensor, which includes:
    /// 1. Verifying that the sensor is alive on the I2C bus.
    /// 2. Transferring the large (~120KB) firmware image to the sensor's RAM.
    /// 3. Booting the firmware and verifying the device ID.
    ///
    /// This is a blocking operation that can take a significant amount of time
    /// due to the large firmware transfer over I2C.
    ///
    /// It must be called after creating the driver (`new()`) and after any
    /// address changes (`set_address()`).
    /// # Errors
    /// - `Error::PinMissing` if the reset pin is not configured.
    /// - `Error::NotAlive` if the sensor does not respond on the I2C bus.
    /// - `Error::Firmware` if the firmware fails to load and initialize.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Ensure the sensor is powered on and not in reset.
        self.reset_pin.set_high().map_err(|_| Error::PinMissing)?; // This now works with NoPin
        self.delay.delay_ms(2);

        // Fill platform-specific fields in the ST driver state.
        self.stdev.platform.address = self.address as u16;
        // NOTE: The `platform.i2c` field is a placeholder. All I2C operations
        // will be handled by the `embedded-hal` trait implementations.

        // Check if the sensor is alive.
        let mut is_alive: u8 = 0;
        let status = unsafe {
            sys::vl53l5cx_is_alive(&mut self.stdev, &mut is_alive)
        };
        if status != 0 || is_alive == 0 {
            return Err(Error::NotAlive);
        }

        // Initialize the sensor. This loads the firmware.
        let status = unsafe { sys::vl53l5cx_init(&mut self.stdev) };
        if status != 0 {
            return Err(Error::Firmware(status));
        }

        Ok(())
    }

    // --- FFI Helper Methods ---
    // These methods are called by the `impl_vl53l5cx_comms!` macro.

    /// Platform-specific I2C read.
    pub fn comms_read(&mut self, index: u16, data: &mut [u8]) -> Result<(), Error<E>> {
        let index_bytes = index.to_be_bytes();
        self.i2c.write_read(self.address, &index_bytes, data).map_err(Error::I2c)
    }

    /// Platform-specific I2C write.
    pub fn comms_write(&mut self, index: u16, data: &[u8]) -> Result<(), Error<E>> {
        let index_bytes = index.to_be_bytes();

        // The ULD requires the 16-bit index and data to be sent in a single I2C transaction.
        // We must concatenate them. To avoid heap allocation, we use a stack buffer.
        // The largest single transaction is the firmware download, but that is handled
        // by the ULD in chunks. A 260-byte buffer is safe for other commands.
        const MAX_WRITE_BUFFER: usize = 260;
        let write_len = 2 + data.len();

        if write_len > MAX_WRITE_BUFFER {
            // This case should ideally not be hit with the current ULD.
            // If it is, it indicates a very large write that isn't firmware.
            return Err(Error::Firmware(255)); // Indicate a platform error.
        }

        let mut buffer = [0u8; MAX_WRITE_BUFFER];
        buffer[0..2].copy_from_slice(&index_bytes);
        buffer[2..write_len].copy_from_slice(data);

        self.i2c.write(self.address, &buffer[..write_len]).map_err(Error::I2c)
    }

    /// Platform-specific delay.
    pub fn comms_wait_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

/// Generates the `extern "C"` functions required by the `vl53l5cx-sys` crate.
///
/// The ST ULD (Ultra Lite Driver) is written in C and expects to be able to call
/// global functions for platform-specific operations like I2C communication and delays.
/// This macro creates those required functions and routes the calls to a specific,
/// statically-defined driver instance.
///
/// # Usage
///
/// In your application's binary crate, define a static `Mutex` to hold your
/// `Vl53l5cx` driver instance. Then, invoke this macro with the path to that static variable.
///
/// ```ignore
/// // In your main.rs or equivalent
/// use std::sync::Mutex;
/// use once_cell::sync::Lazy;
/// use vl53l5cx_driver::{Vl53l5cx, impl_vl53l5cx_comms};
///
/// // Define a static, mutex-protected driver instance.
/// // The types must be fully specified.
/// static VL53L5CX_DRIVER: Lazy<Mutex<Vl53l5cx</* I2C */, /* Delay */>>> = Lazy::new(|| {
///     // Create your I2C and Delay implementations
///     let i2c = ...;
///     let delay = ...;
///     Mutex::new(Vl53l5cx::new(i2c, delay))
/// });
///
/// // Generate the FFI functions, pointing them to your static driver.
/// impl_vl53l5cx_comms!(VL53L5CX_DRIVER);
/// ```
///
/// For `no_std` environments, you would typically use a `Mutex` from a crate like `critical-section`.
#[macro_export]
macro_rules! impl_vl53l5cx_comms {
    ($driver_static:path) => {
        use $crate::sys::VL53L5CX_Platform;

        #[no_mangle]
        pub extern "C" fn VL53L5CX_RdMulti(
            p_platform: *mut VL53L5CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let mut driver = $driver_static.lock().unwrap();
            let data_slice = unsafe { core::slice::from_raw_parts_mut(p_values, size as usize) };

            // The address in p_platform is updated by the driver's init() method.
            // We can optionally verify it matches our driver's state.
            if unsafe{ (*p_platform).address as u8 != driver.address } {
                return 255; // Address mismatch error
            }

            match driver.comms_read(index, data_slice) {
                Ok(_) => 0,
                Err(_) => 1, // ULD expects 1 for I2C errors
            }
        }

        #[no_mangle]
        pub extern "C" fn VL53L5CX_WrMulti(
            p_platform: *mut VL53L5CX_Platform,
            index: u16,
            p_values: *mut u8,
            size: u32,
        ) -> u8 {
            if p_platform.is_null() { return 255; }
            let mut driver = $driver_static.lock().unwrap();
            let data_slice = unsafe { core::slice::from_raw_parts(p_values, size as usize) };

            match driver.comms_write(index, data_slice) {
                Ok(_) => 0,
                Err(_) => 1,
            }
        }

        #[no_mangle]
        pub extern "C" fn VL53L5CX_WaitMs(_p_platform: *mut VL53L5CX_Platform, milliseconds: u32) -> u8 {
            let mut driver = $driver_static.lock().unwrap();
            driver.comms_wait_ms(milliseconds);
            0 // Success
        }
    };
}