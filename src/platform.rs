use embedded_hal::i2c::{I2c, SevenBitAddress};
use std::time::Instant;
use std::ffi::c_void;
use once_cell::sync::Lazy;

/// A newtype wrapper for a generic I2C error to satisfy the orphan rule.
#[derive(Debug)]
pub struct PlatformError(Box<dyn std::error::Error + Send>);

impl<E: std::error::Error + Send + 'static> From<E> for PlatformError {
    fn from(e: E) -> Self {
        Self(Box::new(e))
    }
}

// Now we can legally implement the foreign trait `embedded_hal::i2c::Error`
// for our local type `PlatformError`.
impl embedded_hal::i2c::Error for PlatformError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        // We can't know the specific kind from a generic error, so we return Other.
        embedded_hal::i2c::ErrorKind::Other
    }
}

/// This struct mirrors the C `VL53L5CX_Platform` struct.
/// We must ensure its layout is compatible with the C definition.
#[repr(C)]
pub struct VL53L5CX_Platform {
    pub address: u16,
    pub p_com: *mut c_void,
}

// The ST driver requires a tick counter for timeouts.
// We can implement this with `std::time::Instant`.
static START_TIME: Lazy<Instant> = Lazy::new(Instant::now);

// `vl53l5cx_platform.h` requires these functions to be defined.
// We define them here in Rust with `extern "C"` and `#[no_mangle]`.

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_write(
    p_com: *mut c_void,
    address: u16,
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    // Cast the void pointer back to our I2C bus trait object.
    let i2c = unsafe {
        &mut **(p_com as *mut Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send>)
    };

    // The ST driver uses a 16-bit device address where the 7-bit I2C address
    // is shifted left by 1. We must shift it right to get the real address.
    let i2c_addr = (address >> 1) as u8;

    // The driver wants to write `count` bytes starting from a 16-bit `index`.
    // We need to combine the index and the data into a single buffer for a single I2C transaction.
    let index_bytes = index.to_be_bytes();
    let data_slice = unsafe { std::slice::from_raw_parts(data, count as usize) };

    // Create a temporary buffer for the write operation:
    // [index_high, index_low, data_0, data_1, ..., data_n]
    let mut buffer = Vec::with_capacity(2 + count as usize);
    buffer.extend_from_slice(&index_bytes);
    buffer.extend_from_slice(data_slice);

    // Propagate the I2C error to the C driver.
    // The driver expects 0 for success and non-zero for failure.
    // We'll use 255 as our generic I2C error code.
    match i2c.write(i2c_addr, &buffer) {
        Ok(_) => 0,    // Success
        Err(_) => 255, // Error
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_read(
    p_com: *mut c_void,
    address: u16,
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    // Cast the void pointer back to our I2C bus trait object.
    let i2c = unsafe {
        &mut **(p_com as *mut Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send>)
    };

    let i2c_addr = (address >> 1) as u8;
    let index_bytes = index.to_be_bytes();
    let data_slice = unsafe { std::slice::from_raw_parts_mut(data, count as usize) };

    // To read from a specific register, we first perform a write of the
    // register index, then perform the read.
    // Propagate the I2C error to the C driver.
    match i2c.write_read(i2c_addr, &index_bytes, data_slice) {
        Ok(_) => 0,    // Success
        Err(_) => 255, // Error
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_get_tick() -> u32 {
    // This needs to return a millisecond tick count.
    // NOTE: On a bare-metal system without `std`, you would use a timer peripheral.
    START_TIME.elapsed().as_millis() as u32
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_sleep(milliseconds: u32) -> u8 {
    std::thread::sleep(std::time::Duration::from_millis(milliseconds as u64));
    0 // Success
}
