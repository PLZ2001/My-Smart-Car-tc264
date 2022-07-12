/*
 * COPYRIGHT (C) STMicroelectronics 2014. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/**
 * @file   VL53L0X_platform.h
 * @brief  Function prototype definitions for Ewok Platform layer.
 *
 */


#ifndef _VL53L0X_I2C_PLATFORM_H_
#define _VL53L0X_I2C_PLATFORM_H_

#include "vl53l0x_def.h"

#ifdef __cplusplus
extern "C" {
#endif

// Include uint8, unit16_t  etc definitions

#include <stdint.h>
#include <stdarg.h>


/**
 *  @brief Typedef defining .\n
 * The developer shoud modify this to suit the platform being deployed.
 *
 */

// enum  {TRUE = true, FALSE = false};

/**
 * @brief Typedef defining 8 bit unsigned char type.\n
 * The developer shoud modify this to suit the platform being deployed.
 *
 */

#ifndef bool_t
typedef unsigned char bool_t;
#endif


#define	   I2C                0x01
#define	   SPI                0x00

#define    COMMS_BUFFER_SIZE    64  // MUST be the same size as the SV task buffer

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4

#define    VL53L0X_MAX_STRING_LENGTH_PLT       256

/**
 * @brief  Initialise platform comms.
 *
 * @param  comms_type      - selects between I2C and SPI
 * @param  comms_speed_khz - unsigned short containing the I2C speed in kHz
 *
 * @return status - status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_comms_initialise(uint8  comms_type,
                                          uint16 comms_speed_khz);

/**
 * @brief  Close platform comms.
 *
 * @return status - status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_comms_close(void);

/**
 * @brief  Cycle Power to Device
 *
 * @return status - status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_cycle_power(void);


/**
 * @brief Writes the supplied byte buffer to the device
 *
 * Wrapper for SystemVerilog Write Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8 *spad_enables;
 *
 * int status = VL53L0X_write_multi(RET_SPAD_EN_0, spad_enables, 36);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  pdata - pointer to uint8 buffer containing the data to be written
 * @param  count - number of bytes in the supplied byte buffer
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_write_multi(uint8 address, uint8 index, uint8  *pdata, int32 count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * Wrapper for SystemVerilog Read Multi task
 *
 * @code
 *
 * Example:
 *
 * uint8 buffer[COMMS_BUFFER_SIZE];
 *
 * int status = status  = VL53L0X_read_multi(DEVICE_ID, buffer, 2)
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  pdata - pointer to the uint8 buffer to store read data
 * @param  count - number of uint8's to read
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_read_multi(uint8 address,  uint8 index, uint8  *pdata, int32 count);


/**
 * @brief  Writes a single byte to the device
 *
 * Wrapper for SystemVerilog Write Byte task
 *
 * @code
 *
 * Example:
 *
 * uint8 page_number = MAIN_SELECT_PAGE;
 *
 * int status = VL53L0X_write_byte(PAGE_SELECT, page_number);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  data  - uint8 data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_write_byte(uint8 address,  uint8 index, uint8   data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16 nvm_ctrl_pulse_width = 0x0004;
 *
 * int status = VL53L0X_write_word(NVM_CTRL__PULSE_WIDTH_MSB, nvm_ctrl_pulse_width);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  data  - uin16_t data value write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_write_word(uint8 address,  uint8 index, uint16  data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 * Uses SystemVerilog Write Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32 nvm_data = 0x0004;
 *
 * int status = VL53L0X_write_dword(NVM_CTRL__DATAIN_MMM, nvm_data);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  data  - uint32 data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_write_dword(uint8 address, uint8 index, uint32  data);



/**
 * @brief  Reads a single byte from the device
 *
 * Uses SystemVerilog Read Byte task.
 *
 * @code
 *
 * Example:
 *
 * uint8 device_status = 0;
 *
 * int status = VL53L0X_read_byte(STATUS, &device_status);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index  - uint8 register index value
 * @param  pdata  - pointer to uint8 data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_read_byte(uint8 address,  uint8 index, uint8  *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint16 timeout = 0;
 *
 * int status = VL53L0X_read_word(TIMEOUT_OVERALL_PERIODS_MSB, &timeout);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index  - uint8 register index value
 * @param  pdata  - pointer to uint16 data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_read_word(uint8 address,  uint8 index, uint16 *pdata);


/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 * Uses SystemVerilog Read Multi task.
 *
 * @code
 *
 * Example:
 *
 * uint32 range_1 = 0;
 *
 * int status = VL53L0X_read_dword(RANGE_1_MMM, &range_1);
 *
 * @endcode
 *
 * @param  address - uint8 device address value
 * @param  index - uint8 register index value
 * @param  pdata - pointer to uint32 data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_read_dword(uint8 address, uint8 index, uint32 *pdata);


/**
 * @brief  Implements a programmable wait in us
 *
 * Wrapper for SystemVerilog Wait in micro seconds task
 *
 * @param  wait_us - integer wait in micro seconds
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_platform_wait_us(int32 wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * Wrapper for SystemVerilog Wait in milli seconds task
 *
 * @param  wait_ms - integer wait in milli seconds
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_wait_ms(int32 wait_ms);


/**
 * @brief Set GPIO value
 *
 * @param  level  - input  level - either 0 or 1
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_set_gpio(uint8  level);


/**
 * @brief Get GPIO value
 *
 * @param  plevel - uint8 pointer to store GPIO level (0 or 1)
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_get_gpio(uint8 *plevel);

/**
 * @brief Release force on GPIO
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */

int32 VL53L0X_release_gpio(void);


/**
* @brief Get the frequency of the timer used for ranging results time stamps
*
* @param[out] ptimer_freq_hz : pointer for timer frequency
*
* @return status : 0 = ok, 1 = error
*
*/

int32 VL53L0X_get_timer_frequency(int32 *ptimer_freq_hz);

/**
* @brief Get the timer value in units of timer_freq_hz (see VL53L0X_get_timestamp_frequency())
*
* @param[out] ptimer_count : pointer for timer count value
*
* @return status : 0 = ok, 1 = error
*
*/

int32 VL53L0X_get_timer_value(int32 *ptimer_count);





#ifdef __cplusplus
}
#endif

#endif //_VL53L0X_I2C_PLATFORM_H_

