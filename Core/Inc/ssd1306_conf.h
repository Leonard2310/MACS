#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

#include "stm32f3xx_hal.h"  // serve per i tipi HAL
#include "i2c.h"            // include dove è dichiarato hi2c1

// Specifica la tua famiglia
#define STM32F3

// Usa I2C (non SPI)
#define SSD1306_USE_I2C

// Porta I2C e indirizzo del display
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Include i font necessari
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#endif /* __SSD1306_CONF_H__ */
