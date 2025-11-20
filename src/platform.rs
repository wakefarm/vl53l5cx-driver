#[cfg(not(feature = "std"))]
extern crate alloc;

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::delay::DelayNs; // Use standard delay trait
use core::ffi::c_void;
use core::slice;

#[cfg(feature = "std")]
use std::vec::Vec;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use alloc::boxed::Box;

// 1. Define errors
#[derive(Debug)]
pub struct PlatformError;

impl embedded_hal::i2c::Error for PlatformError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

// 2. Define a container for BOTH I2c and Delay
pub struct PlatformResources {
    pub i2c: Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send>,
    pub delay: Box<dyn DelayNs + Send>,
}

// 3. The C-compatible struct
#[repr(C)]
pub struct VL53L5CX_Platform {
    pub address: u16,
    pub p_com: *mut c_void, // Will point to PlatformResources
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_write(
    p_platform: *mut VL53L5CX_Platform,
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }

        // Cast p_com back to our Rust struct
        let resources = &mut *((*p_platform).p_com as *mut PlatformResources);
        let i2c_addr = (*p_platform).address as u8;
        let index_bytes = index.to_be_bytes();

        // Use heap-less approach for small buffers if possible
        if count <= 256 {
            let mut buffer = [0u8; 258];
            buffer[0] = index_bytes[0];
            buffer[1] = index_bytes[1];
            let data_slice = slice::from_raw_parts(data, count as usize);
            buffer[2..2 + count as usize].copy_from_slice(data_slice);

            match resources.i2c.write(i2c_addr, &buffer[0..2 + count as usize]) {
                Ok(_) => 0,
                Err(_) => 255,
            }
        } else {
            let mut buffer = Vec::with_capacity(2 + count as usize);
            buffer.extend_from_slice(&index_bytes);
            let data_slice = slice::from_raw_parts(data, count as usize);
            buffer.extend_from_slice(data_slice);

            match resources.i2c.write(i2c_addr, &buffer) {
                Ok(_) => 0,
                Err(_) => 255,
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_read(
    p_platform: *mut VL53L5CX_Platform,
    index: u16,
    data: *mut u8,
    count: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }
        
        let resources = &mut *((*p_platform).p_com as *mut PlatformResources);
        let i2c_addr = (*p_platform).address as u8;
        let index_bytes = index.to_be_bytes();
        let data_slice = slice::from_raw_parts_mut(data, count as usize);

        match resources.i2c.write_read(i2c_addr, &index_bytes, data_slice) {
            Ok(_) => 0,
            Err(_) => 255,
        }
    }
}

// Now we can implement sleep generically using the trait!
#[no_mangle]
pub extern "C" fn vl53l5cx_platform_wait_ms(
    p_platform: *mut VL53L5CX_Platform,
    time_ms: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }

        let resources = &mut *((*p_platform).p_com as *mut PlatformResources);
        resources.delay.delay_ms(time_ms);
        0
    }
}

// Legacy functions called by C-driver but redirected 
#[no_mangle]
pub extern "C" fn vl53l5cx_platform_sleep(_milliseconds: u32) -> u8 {
    // This function is tricky because it doesn't take p_platform.
    // However, ST's driver usually calls VL53L5CX_WaitMs which passes p_platform.
    // We modified platform.h to define: #define VL53L5CX_WaitMs(p, time) vl53l5cx_platform_wait_ms(p, time)
    // So this function might not be needed if platform.h is correct.
    0 
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_get_tick() -> u32 {
    0 // Not strictly used by ULD in blocking mode
}