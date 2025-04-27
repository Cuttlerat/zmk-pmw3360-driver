#pragma once

#include <zephyr/drivers/sensor.h>
#include "pixart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timings (in us) used in SPI communication. Since MCU should not do other tasks during wait,
 * k_busy_wait is used instead of k_sleep */
// - sub-us time is rounded to us, due to the limitation of k_busy_wait, see :
// https://github.com/zephyrproject-rtos/zephyr/issues/6498
#define T_NCS_SCLK 1     /* 120 ns (rounded to 1us) */
#define T_SCLK_NCS_WR 10 /* 10 us */
#define T_SRAD 4         /* 4 us */
#define T_SRAD_MOTBR 4   /* same as T_SRAD */
#define T_SRX 1          /* 250 ns (rounded to 1 us) */
#define T_SWX 30         /* SWW: 30 us, SWR: 20 us */
#define T_BEXIT 1        /* 250 ns (rounded to 1us)*/

/* Sensor registers (addresses) for PMW3360 */
#define PMW3360_REG_PRODUCT_ID 0x00
#define PMW3360_REG_REVISION_ID 0x01
#define PMW3360_REG_MOTION 0x02
#define PMW3360_REG_DELTA_X_L 0x03
#define PMW3360_REG_DELTA_Y_L 0x04
#define PMW3360_REG_DELTA_XY_H 0x05
#define PMW3360_REG_SQUAL 0x06
#define PMW3360_REG_RAW_DATA_SUM 0x07
#define PMW3360_REG_MAXIMUM_RAW_DATA 0x08
#define PMW3360_REG_MINIMUM_RAW_DATA 0x09
#define PMW3360_REG_SHUTTER_UPPER 0x0A
#define PMW3360_REG_SHUTTER_LOWER 0x0B

/* Control registers */
#define PMW3360_REG_CONFIG1 0x0F
#define PMW3360_REG_CONFIG2 0x10
#define PMW3360_REG_ANGLE_TUNE 0x11
#define PMW3360_REG_FRAME_CAPTURE 0x12
#define PMW3360_REG_SROM_ENABLE 0x13
#define PMW3360_REG_RUN_DOWNSHIFT 0x14
#define PMW3360_REG_REST1_RATE 0x15
#define PMW3360_REG_REST1_DOWNSHIFT 0x16
#define PMW3360_REG_REST2_RATE 0x17
#define PMW3360_REG_REST2_DOWNSHIFT 0x18
#define PMW3360_REG_REST3_RATE 0x19
#define PMW3360_REG_OBSERVATION 0x24
#define PMW3360_REG_DATA_BURST 0x50
#define PMW3360_REG_POWER_UP_RESET 0x3A
#define PMW3360_REG_SHUTDOWN 0x3B
#define PMW3360_REG_INVERSE_PRODUCT_ID 0x3F
#define PMW3360_REG_MOTION_BURST 0x50
#define PMW3360_REG_SROM_ID 0x2A
#define PMW3360_REG_LIFT_CONFIG 0x63
#define PMW3360_REG_RESOLUTION 0x75

/* Sensor identification values */
#define PMW3360_PRODUCT_ID 0x42

/* Power-up register commands */
#define PMW3360_POWERUP_CMD_RESET 0x5A

/* Max register count readable in a single motion burst */
#define PMW3360_MAX_BURST_SIZE 12

/* Register count used for reading a single motion burst */
#define PMW3360_BURST_SIZE 6

/* Position in the motion registers */
#define PMW3360_X_L_POS 1
#define PMW3360_Y_L_POS 2
#define PMW3360_XY_H_POS 3

/* cpi/resolution range */
#define PMW3360_MAX_CPI 12000
#define PMW3360_MIN_CPI 100

/* write command bit position */
#define SPI_WRITE_BIT BIT(7)

/* Helper macros used to convert sensor values. */
#define PMW3360_SVALUE_TO_CPI(svalue) ((uint32_t)(svalue).val1)
#define PMW3360_SVALUE_TO_TIME(svalue) ((uint32_t)(svalue).val1)

#if defined(CONFIG_PMW3360_POLLING_RATE_250) || defined(CONFIG_PMW3360_POLLING_RATE_125_SW)
#define PMW3360_POLLING_RATE_VALUE 0x0D
#elif defined(CONFIG_PMW3360_POLLING_RATE_125)
#define PMW3360_POLLING_RATE_VALUE 0x00
#else
#error "A valid PMW3360 polling rate must be selected"
#endif

#ifdef CONFIG_PMW3360_FORCE_AWAKE
#define PMW3360_FORCE_MODE_VALUE 0xF0
#else
#define PMW3360_FORCE_MODE_VALUE 0x00
#endif

#define PMW3360_PERFORMANCE_VALUE (PMW3360_FORCE_MODE_VALUE | PMW3360_POLLING_RATE_VALUE)

#ifdef CONFIG_PMW3360_INVERT_SCROLL_X
#define PMW3360_SCROLL_X_NEGATIVE 1
#define PMW3360_SCROLL_X_POSITIVE -1
#else
#define PMW3360_SCROLL_X_NEGATIVE -1
#define PMW3360_SCROLL_X_POSITIVE 1
#endif

#ifdef CONFIG_PMW3360_INVERT_SCROLL_Y
#define PMW3360_SCROLL_Y_NEGATIVE 1
#define PMW3360_SCROLL_Y_POSITIVE -1
#else
#define PMW3360_SCROLL_Y_NEGATIVE -1
#define PMW3360_SCROLL_Y_POSITIVE 1
#endif

#ifdef __cplusplus
}
#endif 