use embedded_hal::i2c::{I2c, SevenBitAddress};
use std::time::Instant;
use std::ffi::c_void;
use std::slice;
use once_cell::sync::Lazy;

/// A newtype wrapper for a generic I2C error to satisfy the orphan rule.

// 1. Define the Trait Object type clearly
// This wraps any I2C bus that implements the Error trait we defined
pub type I2cHandle = Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send>;
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
    p_platform: *mut VL53L5CX_Platform, // <--- FIXED: Accepts struct ptr
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }

        let platform = &*p_platform;
        let i2c = &mut *(platform.p_com as *mut I2cHandle);
        let i2c_addr = platform.address as u8;
        let index_bytes = index.to_be_bytes();

        // OPTIMIZATION: Use stack memory for small writes (up to 256 bytes data + 2 bytes header)
        // This covers 99% of command traffic and avoids malloc/free overhead.
        if count <= 256 {
            let mut buffer = [0u8; 258]; // 256 data + 2 address

            // 1. Copy Index (Big Endian)
            buffer[0] = index_bytes[0];
            buffer[1] = index_bytes[1];

            // 2. Copy Data
            let data_slice = slice::from_raw_parts(data, count as usize);
            buffer[2..2 + count as usize].copy_from_slice(data_slice);

            // 3. Send Stack Buffer
            match i2c.write(i2c_addr, &buffer[0..2 + count as usize]) {
                Ok(_) => 0,
                Err(_) => 255,
            }
        } else {
            // FALLBACK: Use Heap (Vec) for large Firmware chunks (~32KB)
            let mut buffer = Vec::with_capacity(2 + count as usize);
            buffer.extend_from_slice(&index_bytes);
            let data_slice = slice::from_raw_parts(data, count as usize);
            buffer.extend_from_slice(data_slice);

            match i2c.write(i2c_addr, &buffer) {
                Ok(_) => 0,
                Err(_) => 255,
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_read(
    p_platform: *mut VL53L5CX_Platform, // <--- FIXED
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }
        let platform = &*p_platform;
        let i2c = &mut *(platform.p_com as *mut I2cHandle);

        let i2c_addr = platform.address as u8;
        let index_bytes = index.to_be_bytes();
        let data_slice = slice::from_raw_parts_mut(data, count as usize);

        match i2c.write_read(i2c_addr, &index_bytes, data_slice) {
            Ok(_) => 0,
            Err(_) => 255,
        }
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
