use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::delay::DelayNs;

#[cfg(not(feature = "std"))]
extern crate alloc;
use core::slice;
use crate::{Vl53l5cx, PlatformTrait};

#[derive(Debug)]
pub struct PlatformError;

impl embedded_hal::i2c::Error for PlatformError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

// FIX: Remove "Error = PlatformError" bound. 
// We accept ANY I2c implementation, and handle the error mapping internally.
impl<'a, I: I2c<SevenBitAddress> + 'a, D: DelayNs + 'a> PlatformTrait<'a> for Vl53l5cx<'a, I, D> {
    fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), PlatformError> {
        // FIX: Map the generic I::Error to our PlatformError
        self.i2c.write(addr, data).map_err(|_| PlatformError)
    }
    fn write_read(&mut self, addr: u8, wr_buf: &[u8], rd_buf: &mut [u8]) -> Result<(), PlatformError> {
        // FIX: Map the generic I::Error to our PlatformError
        self.i2c.write_read(addr, wr_buf, rd_buf).map_err(|_| PlatformError)
    }
    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }
}

// The C-compatible platform struct definition MUST match the C binding
#[repr(C)]
pub struct VL53L5CX_Platform {
    pub address: u16,
    pub p_com: *mut core::ffi::c_void,
}

/// Helper function to safely retrieve the `dyn PlatformTrait` from `p_com`.
unsafe fn with_platform<F, R>(p_platform: *mut VL53L5CX_Platform, f: F) -> R
where
    F: for<'a> FnOnce(&mut dyn PlatformTrait<'a>) -> R,
{
    // Cast p_com to a pointer-to-fat-pointer
    let proxy_ptr_ptr = (*p_platform).p_com as *mut *mut dyn PlatformTrait<'_>;
    
    // Dereference to get the actual fat pointer to the struct
    let sensor_ptr = *proxy_ptr_ptr;
    
    // Convert raw pointer to mutable reference
    let platform = &mut *sensor_ptr;
    
    f(platform)
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

        with_platform(p_platform, |platform| {
            let i2c_addr = (*p_platform).address as u8;
            let index_bytes = index.to_be_bytes();

            if count <= 256 {
                let mut buffer = [0u8; 258];
                buffer[0] = index_bytes[0];
                buffer[1] = index_bytes[1];
                let data_slice = slice::from_raw_parts(data, count as usize);
                buffer[2..2 + count as usize].copy_from_slice(data_slice);

                match platform.write(i2c_addr, &buffer[0..2 + count as usize]) {
                    Ok(_) => 0,
                    Err(_) => 255,
                }
            } else {
                // Path 2: Large writes (primarily firmware upload)

                #[cfg(feature = "std")]
                {
                    // If std is available, use Vec for the safest single transaction.
                    let mut buffer = std::vec::Vec::with_capacity(2 + count as usize);
                    buffer.extend_from_slice(&index_bytes);
                    let data_slice = slice::from_raw_parts(data, count as usize);
                    buffer.extend_from_slice(data_slice);

                    match platform.write(i2c_addr, &buffer) {
                        Ok(_) => 0,
                        Err(_) => 255,
                    }
                }

                #[cfg(not(feature = "std"))]
                {
                    // If no-std, use the two-call split to avoid allocation.
                    // This relies on the I2C HAL not sending a STOP condition between calls.
                    if platform.write(i2c_addr, &index_bytes).is_err() {
                        return 255;
                    }
                    // This inner unsafe block is required because the closure is a safe context.
                    #[allow(unused_unsafe)]
                    let data_slice = unsafe { core::slice::from_raw_parts(data, count as usize) };
                    if platform.write(i2c_addr, data_slice).is_err() {
                        return 255;
                    }
                    0
                }
            }
        })
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
        
        with_platform(p_platform, |platform| {
            let i2c_addr = (*p_platform).address as u8;
            let index_bytes = index.to_be_bytes();
            let data_slice = slice::from_raw_parts_mut(data, count as usize);

            match platform.write_read(i2c_addr, &index_bytes, data_slice) {
                Ok(_) => 0,
                Err(_) => 255,
            }
        })
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_wait_ms(
    p_platform: *mut VL53L5CX_Platform,
    time_ms: u32,
) -> u8 {
    unsafe {
        if p_platform.is_null() || (*p_platform).p_com.is_null() {
            return 255;
        }

        with_platform(p_platform, |platform| {
            platform.delay_ms(time_ms);
            0
        })
    }
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_sleep(_milliseconds: u32) -> u8 {
    0 
}

#[no_mangle]
pub extern "C" fn vl53l5cx_platform_get_tick() -> u32 {
    0 
}