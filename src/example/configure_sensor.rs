use anyhow::Result;
use embedded_hal::i2c::SevenBitAddress;
use i2cdev::linux::LinuxI2cDev;
use vl53l5cx_ffi::{platform::PlatformError, Vl53l5cx, bindings};

fn main() -> Result<()> {
    // 1. Define Hardware specifics
    const I2C_BUS_PATH: &str = "/dev/i2c-2";
    const SENSOR_ADDR: u8 = 0x29; 

    // 2. Initialize Linux I2C
    println!("Opening I2C bus {}...", I2C_BUS_PATH);
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH, SENSOR_ADDR as u16)
        .map_err(|e| anyhow::anyhow!("Failed to open bus: {}", e))?;

    // 3. Box it for the driver (Type Erasure)
    let i2c_bus: Box<dyn embedded_hal::i2c::I2c<SevenBitAddress, Error = PlatformError> + Send> =
        Box::new(i2c.map_err(PlatformError::from));

    // 4. Initialize Sensor
    // Note: This uploads ~90KB firmware via I2C
    println!("Initializing VL53L5CX (Address 0x{:02X})...", SENSOR_ADDR);
    let mut sensor = Vl53l5cx::new(i2c_bus, SENSOR_ADDR)?;
    println!("Sensor Init Complete.");

    // 5. Verify
    if sensor.is_alive()? {
        println!("Sensor is Alive!");
    } else {
        anyhow::bail!("Sensor did not respond to ping!");
    }

    // 6. Configure for Conveyor (8x8, 15Hz)
    sensor.set_resolution(bindings::VL53L5CX_RESOLUTION_8X8)?;
    sensor.set_ranging_frequency_hz(15)?;
    
    println!("Configuration applied. Ready for Volume Analysis.");
    Ok(())
}