/* SSD1306 is an OLED display driver and controller. For more details see: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf */

/* Example of use:
    SSD1306_t display; 

    ssd1306_init(&display);

    ssd1306_clear(&display);
    ssd1306_draw_string(&display, 0, 0, "some text");
    ssd1306_update_display(&display);
*/
    
#ifndef SSD1306_H
#define SSD1306_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>

/* Display settings */
#define SSD1306_WIDTH 	128
#define SSD1306_HEIGHT 	64
#define SSD1306_PAGES 	(SSD1306_HEIGHT / 8)

/* I2C address of SSD1306 device */
#define SSD1306_I2C_ADDR 0x3C

/* SSD1306 control commands */
#define SSD1306_CONTROL_COMMAND      0x00
#define SSD1306_CONTROL_DISPLAY_DATA 0x40

/* SSD1306 commands */
#define SSD1306_DISPLAY_ON         0xAF  // Turn the display on
#define SSD1306_DISPLAY_OFF        0xAE  // Turn the display off
#define SSD1306_CONTRAST           0x81  // Set contrast level
#define SSD1306_NORMAL_DISPLAY     0xA6  // Set normal display (un-inverted)
#define SSD1306_INVERT_DISPLAY     0xA7  // Set inverted display (black-on-white)
#define SSD1306_DISPLAY_ALL_ON     0xA5  // Turn all pixels on
#define SSD1306_DISPLAY_ALL_OFF    0xA4  // Resume to normal display (disable all pixels on)
#define SSD1306_SET_MEMORY_MODE    0x20  // Set memory addressing mode (0x00: horizontal, 0x01: vertical, 0x02: page)
#define SSD1306_COLUMN_ADDR        0x21  // Set column address (start and end)
#define SSD1306_PAGE_ADDR          0x22  // Set page address (start and end)
#define SSD1306_SET_START_LINE     0x40  // Set display start line (0x40-0x7F)
#define SSD1306_SET_SEG_REMAP      0xA0  // Set segment re-map (0xA0: normal, 0xA1: remap)
#define SSD1306_COM_SCAN_INC       0xC0  // Set COM output scan direction (normal, 0xC0)
#define SSD1306_COM_SCAN_DEC       0xC8  // Set COM output scan direction (remapped, 0xC8)
#define SSD1306_SET_DISPLAY_OFFSET 0xD3  // Set display offset
#define SSD1306_SET_COM_PINS       0xDA  // Set COM pins hardware configuration
#define SSD1306_SET_VCOM_DETECT    0xDB  // Set VCOMH deselect level
#define SSD1306_SET_DISPLAY_CLOCK  0xD5  // Set display clock divide ratio/oscillator frequency
#define SSD1306_SET_PRECHARGE      0xD9  // Set pre-charge period
#define SSD1306_SET_MULTIPLEX      0xA8  // Set multiplex ratio (number of display rows)
#define SSD1306_CHARGE_PUMP        0x8D  // Enable or disable charge pump (external or internal power supply)
#define SSD1306_NOP                0xE3  // No operation (used for delays or padding)

/* Default GPIOx IDs of I2C0 pins on Raspberry Pico */
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

/* Structure for SSD1306 I2C buffer */
typedef struct 
{
    uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
} SSD1306_t;

/* SSD1306 functions available to the user */
void ssd1306_init(SSD1306_t* display);
void ssd1306_clear(SSD1306_t* display);
void ssd1306_update_display(SSD1306_t* display);
void ssd1306_draw_char(SSD1306_t* display, uint8_t x, uint8_t y, char c);
void ssd1306_draw_string(SSD1306_t* display, uint8_t x, uint8_t y, const char* str);
void ssd1306_draw_pixel(SSD1306_t* display, uint8_t x, uint8_t y, bool on);

#endif


