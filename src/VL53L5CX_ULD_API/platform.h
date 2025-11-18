/*
 *
 * Copyright (c) 2022, STMicroelectronics - All Rights Reserved
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef VL53L5CX_PLATFORM_H_
#define VL53L5CX_PLATFORM_H_

#include <stdint.h>
#include <string.h>

typedef struct
{
	uint16_t address;
} VL53L5CX_Platform;

uint8_t vl53l5cx_platform_init(VL53L5CX_Platform *p_platform);
uint8_t vl53l5cx_platform_terminate(VL53L5CX_Platform *p_platform);
uint8_t vl53l5cx_platform_write(VL53L5CX_Platform *p_platform, uint16_t address, uint8_t *p_values, uint32_t size);
uint8_t vl53l5cx_platform_read(VL53L5CX_Platform *p_platform, uint16_t address, uint8_t *p_values, uint32_t size);
uint8_t vl53l5cx_platform_wait_ms(VL53L5CX_Platform *p_platform, uint32_t time_ms);

#endif // VL53L5CX_PLATFORM_H_
