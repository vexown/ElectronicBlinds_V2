# AT24C32 EEPROM Example

This example demonstrates how to interface with the Atmel AT24C32 serial EEPROM using the Raspberry Pi Pico 2 W. The code initializes the EEPROM over I2C, then exercises the three core operations: byte write/read, page write/sequential read, and string storage.

## Hardware Requirements

- Raspberry Pi Pico 2 W
- AT24C32 EEPROM module (commonly found integrated on DS3231 RTC breakout boards)
- Jumper wires
- Breadboard (optional)

## Connections

| Pico Pin | AT24C32 Pin | Description  |
|----------|-------------|--------------|
| GPIO 4   | SDA         | I2C Data     |
| GPIO 5   | SCL         | I2C Clock    |
| 3.3V     | VCC         | Power        |
| GND      | GND         | Ground       |
| GND      | A0, A1, A2  | Address pins (all low → I2C address 0x50) |
| GND      | WP          | Write protect disabled (full memory writable) |

> **Note:** The AT24C32 is often found on the same breakout board as the DS3231 RTC, sharing the I2C bus. Both can be connected to the same SDA/SCL lines simultaneously.

## Building the Example

1. Navigate to the root directory of the project:

   ```
   cd /path/to/UnitApplication
   ```

2. Build the example using the provided build script:

   ```
   ./Build.sh --example=AT24C32_EEPROM
   ```

3. The compiled binary files will be available in the output directory.

4. Flash the binary to your Pico using either the UF2 bootloader or the Flash_Application.sh script.

## Expected Output

```
AT24C32 EEPROM Example
----------------------

Initializing AT24C32...
AT24C32 initialized successfully.

--- Byte Write / Read ---
  Writing 0xA5 to address 0x000...
  [OK]  WriteByte
  [OK]  ReadByte
  Read back: 0xA5 — MATCH

--- Page Write / Sequential Read ---
  Writing 8 bytes to address 0x020...
  [OK]  WritePage
  [OK]  ReadSequential
  Written: 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
  Read:    0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08
  Result:  MATCH

--- String Write / Read ---
  Writing string "Hello!" to address 0x040...
  [OK]  WritePage
  [OK]  ReadSequential
  Read back: "Hello!" — MATCH

Done.
```

## Code Explanation

The example exercises three scenarios that cover the full driver API:

- **Byte write / read** — writes a single byte (`0xA5`) to address `0x000` using `AT24C32_WriteByte`, then reads it back with `AT24C32_ReadByte` and verifies the value.
- **Page write / sequential read** — writes an 8-byte array to the start of the second page (`0x020`) using `AT24C32_WritePage`, then reads all 8 bytes back in one transaction using `AT24C32_ReadSequential`.
- **String storage** — demonstrates a practical use case by writing a null-terminated string to `0x040` and reading it back.

All write operations block internally until the EEPROM's self-timed write cycle completes (acknowledge polling, tWR ≤ 10 ms), so no manual delays are needed between writes and reads.

## Key Concepts

**Internal write cycle (tWR):** After every byte or page write, the EEPROM performs a self-timed internal clear/write cycle for up to 10 ms. During this period it ignores all I2C traffic. The driver handles this automatically by polling the I2C acknowledge after each write.

**Page boundaries:** The AT24C32 is organised as 128 pages of 32 bytes each. A page write must not cross a page boundary — if it does, the internal address counter wraps within the current page, overwriting earlier bytes silently. `AT24C32_WritePage` validates this and returns `AT24C32_ERROR_INVALID_ADDR` if a boundary would be crossed.

**Data retention:** Once written, data is retained for a minimum of 100 years without power, making the AT24C32 suitable for storing configuration, calibration values, or timestamps.

## Troubleshooting

1. **"AT24C32 not found on I2C bus"**
   - Verify SDA/SCL wiring and that the module is powered.
   - Confirm the I2C address is 0x50 (A0, A1, A2 all tied to GND).
   - If sharing the bus with a DS3231, ensure pull-up resistors are present on the I2C lines (the DS3231 module usually includes them).

2. **MISMATCH on read-back**
   - Check that the WP pin is tied to GND (write protect disabled).
   - Ensure the power supply is stable during the write cycle.

3. **`AT24C32_ERROR_INVALID_ADDR` on `WritePage`**
   - The write would have crossed a 32-byte page boundary. Align the start address to a multiple of 32, or split the write into two calls.
