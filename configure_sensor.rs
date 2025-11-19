use anyhow::Result;
use embedded_hal::i2c::SevenBitAddress;
use i2cdev::linux::{LinuxI2CError, LinuxI2cDev};
use vl53l5cx_ffi::{platform::PlatformError, Vl53l5cx, bindings};

/// This example demonstrates how to initialize, configure, and check the VL53L5CX sensor.
fn main() -> Result<()> {
    // --- Platform-specific I2C setup ---
    // On BeagleBone Black, the I2C bus is typically /dev/i2c-2.
    // On Windows/for development, you might use a mock or a different I2C implementation.
    const I2C_BUS_PATH: &str = "/dev/i2c-2";
    let i2c = LinuxI2cDev::new(I2C_BUS_PATH, vl53l5cx_ffi::platform::DEFAULT_I2C_ADDRESS)
        .map_err(|e| anyhow::anyhow!("Failed to open I2C bus: {}", e))?;

    // The platform layer requires a boxed I2C object that implements the embedded-hal traits.
    // We also need to wrap the error type in our `PlatformError` newtype.
    let i2c_bus: Box<dyn embedded_hal::i2c::I2c<SevenBitAddress, Error = PlatformError> + Send> =
        Box::new(i2c.map_err(PlatformError::from));

    println!("Initializing sensor...");

    // --- Initialize the driver ---
    let mut sensor = Vl53l5cx::new(i2c_bus)?;
    println!("Sensor initialized.");

    // --- Check if the sensor is alive ---
    let is_alive = sensor.is_alive()?;
    if !is_alive {
        anyhow::bail!("Sensor is not alive!");
    }
    println!("Sensor is alive!");

    // --- Configure the sensor for the silage project ---
    println!("Setting resolution to 8x8...");
    sensor.set_resolution(bindings::VL53L5CX_RESOLUTION_8X8)?;
    println!("Resolution set.");

    // For 8x8 resolution, the maximum ranging frequency is 15 Hz.
    let ranging_freq_hz = 15;
    println!("Setting ranging frequency to {} Hz...", ranging_freq_hz);
    sensor.set_ranging_frequency_hz(ranging_freq_hz)?;
    println!("Ranging frequency set.");

    println!("\nSensor configured successfully!");

    Ok(())
}