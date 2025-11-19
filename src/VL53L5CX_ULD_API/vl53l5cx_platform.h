/**
 * @file  vl53l5cx_platform.h
 * @brief Platform-specific function declarations for the VL53L5CX ULD.
 *
 * This file declares the functions that the C driver requires for I2C access
 * and timing. These functions are implemented in Rust in `src/platform.rs`.
 */

#ifndef VL53L5CX_PLATFORM_H_
#define VL53L5CX_PLATFORM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Writes `count` bytes of data to the sensor at a given 16-bit `index`.
 * @param dev_addr The 16-bit device address (7-bit address << 1).
 * @param index    The 16-bit register index to write to.
 * @param data     Pointer to the data buffer to write.
 * @param count    Number of bytes to write.
 * @return 0 on success, non-zero on failure.
 */
uint8_t vl53l5cx_platform_write(uint16_t dev_addr, uint16_t index, uint8_t *data, uint32_t count);

/**
 * @brief Reads `count` bytes of data from the sensor at a given 16-bit `index`.
 * @param dev_addr The 16-bit device address (7-bit address << 1).
 * @param index    The 16-bit register index to read from.
 * @param data     Pointer to the buffer to store the read data.
 * @param count    Number of bytes to read.
 * @return 0 on success, non-zero on failure.
 */
uint8_t vl53l5cx_platform_read(uint16_t dev_addr, uint16_t index, uint8_t *data, uint32_t count);

/**
 * @brief Gets the current system tick count in milliseconds.
 * @return A 32-bit millisecond tick count.
 */
uint32_t vl53l5cx_platform_get_tick(void);

/**
 * @brief Pauses execution for a specified number of milliseconds.
 * @param milliseconds The number of milliseconds to sleep.
 * @return 0 on success.
 */
uint8_t vl53l5cx_platform_sleep(uint32_t milliseconds);

#ifdef __cplusplus
}
#endif

#endif // VL53L5CX_PLATFORM_H_