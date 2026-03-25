#include "SSD1306_HAL.h"
#include "I2C_HAL.h"

/* Structure for I2C config of SSD1306 */
static I2C_Config i2c_cfg = 
{
    .instance = I2C_INSTANCE_0,
    .sda_pin = I2C_SDA_PIN,
    .scl_pin = I2C_SCL_PIN,
    .speed_hz = 400000  // 400 kHz
};

/**
 * font_5x7 - Character font data for 5x7 pixel display on SSD1306
 *
 * This array stores pixel data for a 5x7 font, where each character
 * is represented by a series of bytes. Each byte in a character's 
 * array defines a vertical column of pixels (from top to bottom),
 * with each bit (1 or 0) representing the on/off state of each pixel.
 *
 * For example, {0x7E, 0x11, 0x11, 0x11, 0x7E} defines the letter 'A':
 * - Each byte corresponds to one column in a 5x7 grid.
 * - 1 bits (e.g., 0x7E = 01111110) turn pixels on, forming the shape.
 * - Reading left to right, this arrangement creates a recognizable 'A'.
 *
 * This array allows characters to be easily mapped to the display
 * by sending column data to the SSD1306, enabling custom text output.
 *
 * Example: '!' character represented with 5x7 font:
 *                  0 0 1 0 0 
 *                  0 0 1 0 0 
 *                  0 0 1 0 0 
 *                  0 0 1 0 0 
 *                  0 0 1 0 0 
 *                  0 0 0 0 0 
 *                  0 0 1 0 0 
 */
static const uint8_t font_5x7[][5] = 
{
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32: Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33: !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34: "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35: #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36: $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37: %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38: &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39: '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40: (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41: )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42: *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43: +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44: ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45: -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46: .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47: /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48: 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49: 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50: 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51: 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52: 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53: 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54: 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55: 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56: 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57: 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58: :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59: ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60: <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61: =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62: >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63: ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64: @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65: A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66: B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67: C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68: D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69: E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70: F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71: G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72: H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73: I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74: J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75: K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76: L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // 77: M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78: N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79: O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80: P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81: Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82: R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83: S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84: T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85: U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86: V
    {0x3F, 0x40, 0x30, 0x40, 0x3F}, // 87: W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88: X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // 89: Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90: Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91: [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92: backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93: ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94: ^
    {0x80, 0x80, 0x80, 0x80, 0x80}, // 95: _
    {0x00, 0x03, 0x05, 0x00, 0x00}, // 96: `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97: a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98: b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99: c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100: d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101: e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102: f
    {0x08, 0x14, 0x54, 0x54, 0x3C}, // 103: g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104: h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105: i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 106: j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107: k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108: l
    {0x7C, 0x04, 0x78, 0x04, 0x78}, // 109: m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110: n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111: o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 112: p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 113: q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114: r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115: s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116: t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117: u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118: v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119: w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120: x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 121: y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122: z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123: {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124: |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125: }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 126: ~
};


/* SSD1306 initialization command sequence */
static const uint8_t ssd1306_init_commands[] = 
{
    0xAE, // Display off
    0xD5, 0x80, // Set display clock divide ratio
    0xA8, SSD1306_HEIGHT - 1, // Set multiplex
    0xD3, 0x00, // Set display offset
    0x40, // Set display start line to 0
    0x8D, 0x14, // Enable charge pump
    0x20, 0x00, // Set memory addressing mode to horizontal
    0xA1, // Set segment re-map
    0xC8, // Set COM output scan direction
    0xDA, 0x12, // Set COM pins hardware configuration
    0x81, 0x7F, // Set contrast control
    0xD9, 0xF1, // Set pre-charge period
    0xDB, 0x40, // Set VCOMH deselect level
    0xA4, // Enable display output GDDRAM
    0xA6, // Set normal display mode (A7 for inverse)
    0xAF // Display on
};

/* 
 * Description: Helper function to send a command to the SSD1306 display via I2C.
 * 
 * The function sends a command byte to the SSD1306 OLED display. 
 * The first byte in the transmitted data is the **control byte** (0x00), which tells the SSD1306 
 * that the following byte will be a command (as opposed to data). This control byte is part of 
 * the I2C protocol for the SSD1306. The second byte is the actual **command** to be executed by 
 * the display. The command is passed as a parameter to this function.
 *
 * Parameters:
 *      - command: A single byte representing the command to send to the SSD1306 
 *                (see the SSD1306 datasheet for a list of valid commands). 
 * Returns: void
 */
static void ssd1306_send_command(uint8_t command) 
{
    I2C_WriteByte(i2c_cfg.instance, SSD1306_I2C_ADDR, (uint8_t)SSD1306_CONTROL_COMMAND, command); // 0x00 in this case is not a reg_addr, it's the control byte indicating a command 
}

/* Description: Init the I2C pins and run the initialization sequence of the SSD1306 
 * Parameters: 
 *      - *display: pointer to the main structure of SSD1306's buffer 
 * Returns: void
 */
void ssd1306_init(SSD1306_t* display) 
{
    I2C_Init(&i2c_cfg);
     
    /* Execute the initialization procedure */
    ssd1306_clear(display);
    for (uint8_t i = 0; i < sizeof(ssd1306_init_commands); i++)
    {
        ssd1306_send_command(ssd1306_init_commands[i]);
    }
    ssd1306_clear(display);
    ssd1306_update_display(display);
}

/*
 * Description: Updates the SSD1306 OLED display with the current buffer content.
 * 
 * This function transfers the contents of the display buffer to the SSD1306 screen. 
 * It first sets the column and page address ranges to define the area that will be updated 
 * (covering the entire screen by default). Then, it sends the display data from 
 * `display->buffer` over I2C in pages, where each page represents 8 rows of pixels.
 * 
 * Steps:
 * 1. Sends the **Column Address** command (0x21) to specify the start and end columns 
 *    for the display update. Here, 0 is the start column, and `SSD1306_WIDTH - 1` is the end.
 * 2. Sends the **Page Address** command (0x22) to set the start and end pages. 
 *    The page range is set from 0 to `(SSD1306_HEIGHT / 8) - 1`, covering all display rows.
 * 3. Initializes a data buffer for sending display data. The first byte in each buffer (0x40) 
 *    is the **control byte** indicating that the following bytes are display data.
 * 4. Iterates over each page (8-pixel-high rows) and each column in `display->buffer`, 
 *    copying the display data to `buffer` for the current page.
 * 5. Sends each page's data over I2C to the SSD1306.
 * 
 * Parameters:
 *      - display: Pointer to an SSD1306_t structure holding the display buffer and settings.
 * 
 * Returns: void
 */
void ssd1306_update_display(SSD1306_t* display) 
{
    ssd1306_send_command(0x21); // Set Column Address command
    ssd1306_send_command(0);    // Start column address (0)
    ssd1306_send_command(SSD1306_WIDTH - 1); // End column address (SSD1306_WIDTH - 1)
    ssd1306_send_command(0x22); // Set Page Address command
    ssd1306_send_command(0);    // Start page address (0)
    ssd1306_send_command((SSD1306_HEIGHT / 8) - 1); // End page address

    uint8_t buffer[SSD1306_WIDTH + 1];
    buffer[0] = SSD1306_CONTROL_DISPLAY_DATA;  // Control byte indicating display data
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) 
    {
        for (uint8_t col = 0; col < SSD1306_WIDTH; col++) 
        {
            buffer[col + 1] = display->buffer[page * SSD1306_WIDTH + col];
        }
        I2C_WriteMultiple(i2c_cfg.instance, SSD1306_I2C_ADDR, buffer[0], &buffer[1], SSD1306_WIDTH);
    }
}

/*
 * Description: Clears the SSD1306 display buffer.
 * 
 * This function resets all pixels in the display buffer to "off" by setting 
 * every byte in `display->buffer` to 0. Since the buffer represents the 
 * pixel data for the SSD1306 OLED display, clearing the buffer effectively 
 * blanks the screen when the buffer is sent to the display using 
 * `ssd1306_update_display`.
 * 
 * Parameters:
 *      - display: Pointer to an SSD1306_t structure containing the display buffer.
 * 
 * Returns: void
 * 
 * Note: After calling this function, the display will remain unchanged until 
 * `ssd1306_update_display` is called to send the cleared buffer to the SSD1306 screen.
 */
void ssd1306_clear(SSD1306_t* display) 
{
    memset(display->buffer, 0, sizeof(display->buffer));  // Set all pixels to "off" (0)
}

/*
 * Description: Sets or clears a specific pixel in the SSD1306 display buffer.
 * 
 * This function modifies the display buffer to turn a single pixel "on" or "off"
 * at a specified (x, y) coordinate. The SSD1306 OLED display uses a buffer where
 * each byte represents a vertical column of 8 pixels. This function calculates 
 * the appropriate byte in the buffer for the given pixel and sets or clears 
 * the bit corresponding to the pixel’s y-position.
 * 
 * Parameters:
 *      - display: Pointer to an SSD1306_t structure containing the display buffer.
 *      - x: Horizontal position (column) of the pixel, in the range 0 to SSD1306_WIDTH - 1.
 *      - y: Vertical position (row) of the pixel, in the range 0 to SSD1306_HEIGHT - 1.
 *      - on: Boolean indicating pixel state (true to turn the pixel "on"; false to turn it "off").
 * 
 * Returns: void
 * 
 * Operation:
 *      - Checks if the specified coordinates are within display bounds. If they are not, 
 *        the function exits without modifying the buffer.
 *      - Calculates `byte_idx`, the index in the display buffer where the pixel data 
 *        for the (x, y) coordinate is stored.
 *      - Creates `bit_mask`, a bitmask that targets the specific bit within the byte 
 *        that represents the pixel's y-position.
 *      - If `on` is true, the function sets the bit in `display->buffer[byte_idx]` 
 *        using bitwise OR to turn the pixel on.
 *      - If `on` is false, the function clears the bit using bitwise AND with 
 *        the inverted `bit_mask` to turn the pixel off.
 * 
 * Note: This function only modifies the buffer; to update the display, 
 * `ssd1306_update_display` must be called.
 */
void ssd1306_draw_pixel(SSD1306_t* display, uint8_t x, uint8_t y, bool on) 
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;  // Ensure coordinates are within bounds

    uint16_t byte_idx = (uint16_t)(x + (y / 8) * SSD1306_WIDTH);  // Calculate buffer index for (x, y)
    uint8_t bit_mask = 1 << (y % 8);  // Create a bitmask for the pixel's position within the byte

    if (on) {
        display->buffer[byte_idx] |= bit_mask;  // Set the bit to turn the pixel on
    } else {
        display->buffer[byte_idx] &= (uint8_t)(~bit_mask);  // Clear the bit to turn the pixel off
    }
}

/*
 * Description: Draws a single character on the SSD1306 display at a specified (x, y) position.
 * 
 * This function uses a 5x7 font array (`font_5x7`) where each character is represented by
 * a 5-byte column array. Each byte contains a 7-bit pattern for the vertical pixels of that 
 * column. This function reads the appropriate column data for the given character, 
 * then iterates through the bits to determine whether each pixel should be set "on" or "off" 
 * on the display buffer.
 * 
 * Parameters:
 *      - display: Pointer to an SSD1306_t structure containing the display buffer.
 *      - x: Horizontal position (column) to start drawing the character.
 *      - y: Vertical position (row) to start drawing the character.
 *      - c: Character to be drawn (ASCII values between 32 and 126 only).
 * 
 * Returns: void
 * 
 * Operation:
 *      - Verifies the character is within the printable ASCII range (32-126).
 *      - Retrieves the 5-column font data for the specified character from `font_5x7`.
 *      - For each column, it extracts the 7-bit pixel pattern, and for each bit, 
 *        it calculates if the pixel should be "on" or "off".
 *      - Calls `ssd1306_draw_pixel` to set each pixel in the display buffer accordingly.
 * 
 * Note: This function only modifies the buffer; to display the character, 
 * `ssd1306_update_display` must be called afterward.
 */
void ssd1306_draw_char(SSD1306_t* display, uint8_t x, uint8_t y, char c) 
{
    if (c < 32 || c > 126) return;  // Ensure character is printable ASCII

    const uint8_t* char_data = font_5x7[c - 32];  // Retrieve font data for the character
    for (uint8_t col = 0; col < 5; col++) 
    {
        uint8_t pixels = char_data[col];  // Get pixel data for the column
        for (uint8_t row = 0; row < 7; row++) 
        {
            bool on = pixels & (1 << row);  // Check if each bit in the column is "on" or "off"
            ssd1306_draw_pixel(display, x + col, y + row, on);  // Draw each pixel
        }
    }
}

/*
 * Description: Draws a string of characters on the SSD1306 display, starting at a specified (x, y) position.
 * 
 * This function writes a string of characters by calling `ssd1306_draw_char` for each character
 * in the string. After each character is drawn, the x-position is incremented to leave space 
 * for the next character, accounting for the character width (5 pixels) and a 1-pixel spacing 
 * between characters.
 * 
 * Parameters:
 *      - display: Pointer to an SSD1306_t structure containing the display buffer.
 *      - x: Initial horizontal position (column) to start drawing the string.
 *      - y: Initial vertical position (row) to start drawing the string.
 *      - str: Pointer to the null-terminated string to be drawn.
 * 
 * Returns: void
 * 
 * Operation:
 *      - Iterates through each character in `str`.
 *      - For each character, calls `ssd1306_draw_char` to render it at the current (x, y) position.
 *      - After drawing each character, advances `x` by 6 (5 pixels for the character width plus 
 *        1 pixel for spacing) to position for the next character.
 * 
 * Note: This function modifies the display buffer only; call `ssd1306_update_display` to show 
 * the drawn string on the display.
 */
void ssd1306_draw_string(SSD1306_t* display, uint8_t x, uint8_t y, const char* str) 
{
    while (*str) 
    {
        ssd1306_draw_char(display, x, y, *str++);  // Draw character and move to the next
        x += 6;  // Move x-position right for the next character (5-pixel character width + 1-pixel space)
    }
}


