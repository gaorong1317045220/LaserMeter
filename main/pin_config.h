#pragma once

#include "driver/gpio.h"

// I2C
#define PIN_I2C_SDA        GPIO_NUM_1
#define PIN_I2C_SCL        GPIO_NUM_2

// BNO086
#define PIN_BNO086_INT     GPIO_NUM_10

// Battery ADC
#define PIN_BAT_ADC        GPIO_NUM_4

// SD 1-bit SDMMC
#define PIN_SD_D0          GPIO_NUM_5
#define PIN_SD_CLK         GPIO_NUM_6
#define PIN_SD_CMD         GPIO_NUM_7

// TK018F3716 / NV3030B LCD, merchant single-data-line SPI protocol.
// Commands are framed on D0/MOSI as 02 00 CMD 00, so D1 is not a D/C line.
#define PIN_LCD_CS         GPIO_NUM_19
#define PIN_LCD_SCLK       GPIO_NUM_8
#define PIN_LCD_MOSI       GPIO_NUM_17
#define PIN_LCD_D1         GPIO_NUM_18
// Compatibility alias for the disabled legacy GC9307 diagnostic block.
#define PIN_LCD_DC         PIN_LCD_D1

// Laser ranging module UART
#define PIN_LASER_TX       GPIO_NUM_16
#define PIN_LASER_RX       GPIO_NUM_15


// Camera OV5640 DVP
#define PIN_CAM_XCLK       GPIO_NUM_13
#define PIN_CAM_PCLK       GPIO_NUM_45
#define PIN_CAM_VSYNC      GPIO_NUM_14
#define PIN_CAM_HREF       GPIO_NUM_21
#define PIN_CAM_RST        GPIO_NUM_11
#define PIN_CAM_D0         GPIO_NUM_41
#define PIN_CAM_D1         GPIO_NUM_39
#define PIN_CAM_D2         GPIO_NUM_38
#define PIN_CAM_D3         GPIO_NUM_40
#define PIN_CAM_D4         GPIO_NUM_42
#define PIN_CAM_D5         GPIO_NUM_48
#define PIN_CAM_D6         GPIO_NUM_47
#define PIN_CAM_D7         GPIO_NUM_12

// Debug UART through CH340
#define PIN_DEBUG_TXD      GPIO_NUM_43
#define PIN_DEBUG_RXD      GPIO_NUM_44

// BOOT / EN
#define PIN_BOOT           GPIO_NUM_0

// PCA9557
#define PCA9557_ADDR       0x18
#define PCA_IO_KEY_MEASURE 1
#define PCA_IO_KEY_BACK    2
#define PCA_IO_KEY_OK      0
#define KEY_ACTIVE_LOW     1

// Expected I2C addresses
#define I2C_ADDR_BNO086    0x4A
#define I2C_ADDR_BNO086_ALT 0x4B
#define I2C_ADDR_TOUCH     0x15
#define I2C_ADDR_OV5640    0x3C

// LCD configuration
#define LCD_DRIVER_NV3030B 1
#define LCD_WIDTH          240
#define LCD_HEIGHT         284
#define LCD_X_OFFSET       0
#define LCD_Y_OFFSET       0
#define LCD_SPI_HOST       SPI2_HOST
// Start conservatively for the FPC-connected panel. Once the link is proven
// stable this can be raised in a separate test (merchant reference: 40 MHz).
#define LCD_SPI_CLOCK_HZ   (20 * 1000 * 1000)

// Touch coordinate options
#define TOUCH_SWAP_XY      0
#define TOUCH_INVERT_X     0
#define TOUCH_INVERT_Y     1

// UARTs
#define LASER_UART_NUM     UART_NUM_2
#define LASER_BAUD         38400

// Battery divider verified on the assembled board: 1:1, so GPIO4 sees
// one half of the battery voltage.
#define BAT_DIV_RATIO      2.0f
// Battery-only measurement on 2026-07-17: ADC=2.053 V while a multimeter
// measured 4.160 V at the cell.  Keep the physical divider explicit and use
// this small gain for the assembled-board ADC/resistor tolerance.
#define BAT_ADC_CAL_GAIN   1.01315f
#define BAT_VOLTAGE_SCALE  (BAT_DIV_RATIO * BAT_ADC_CAL_GAIN)
