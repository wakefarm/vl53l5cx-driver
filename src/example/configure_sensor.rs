use anyhow::Result;
use embedded_hal::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};
use i2cdev::linux::LinuxI2cDev;
use vl53l5cx_ffi::{platform::PlatformError, Vl53l5cx, bindings};

/// A simple delay provider for `std` environments that uses `thread::sleep`.
struct StdDelay;

impl DelayNs for StdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let duration = std::time::Duration::from_nanos(ns as u64);
        std::thread::sleep(duration);
    }
}

fn main() -> Result<()> {
    // 1. Define Hardware specifics
    const I2C_BUS_PATH: &str = "/dev/i2c-2";
    const SENSOR_ADDR: u8 = 0x29; 

    // 2. Initialize platform-specific peripherals
    println!("Opening I2C bus {}...", I2C_BUS_PATH);
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH, SENSOR_ADDR)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;
    let delay = StdDelay;

    // 3. Box peripherals for the driver (Type Erasure)
    let i2c_bus: Box<dyn I2c<SevenBitAddress, Error = PlatformError> + Send> =
        Box::new(i2c.map_err(|_| PlatformError));
    let delay_bus: Box<dyn DelayNs + Send> = Box::new(delay);

    // 4. Initialize Sensor
    // Note: This uploads ~90KB firmware via I2C
    println!("Initializing VL53L5CX (Address 0x{:02X})...", SENSOR_ADDR);
    let mut sensor = Vl53l5cx::new(i2c_bus, delay_bus, SENSOR_ADDR)?;
    println!("Sensor Init Complete.");

    // 5. Verify
    if sensor.is_alive()? {
        println!("Sensor is Alive!");
    } else {
        // Using anyhow::bail! is a convenient way to return an error.
        anyhow::bail!("Sensor at address 0x{:02X} did not respond to ping!", SENSOR_ADDR);
    }

    // 6. Configure for Conveyor (8x8, 15Hz)
    sensor.set_resolution(bindings::VL53L5CX_RESOLUTION_8X8)?;
    sensor.set_ranging_frequency_hz(15)?;
    
    println!("Configuration applied. Ready for Volume Analysis.");
    Ok(())
}