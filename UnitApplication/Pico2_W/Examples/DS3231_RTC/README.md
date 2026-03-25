# DS3231 RTC Example

This example demonstrates how to interface with the Maxim DS3231 real-time clock module using the Raspberry Pi Pico 2 W. The code initializes the RTC over I2C, sets an initial date and time, then continuously reads and prints the current time once per second.

## Hardware Requirements

- Raspberry Pi Pico 2 W
- DS3231 RTC module (with CR2032 backup battery fitted)
- Jumper wires
- Breadboard (optional)

## Connections

| Pico Pin | DS3231 Pin | Description    |
|----------|------------|----------------|
| GPIO 4   | SDA        | I2C Data       |
| GPIO 5   | SCL        | I2C Clock      |
| 3.3V     | VCC        | Power          |
| GND      | GND        | Ground         |

> **Note:** The DS3231 module typically includes onboard I2C pull-up resistors. If yours does not, add 4.7kΩ resistors from SDA and SCL to 3.3V.

## Building the Example

1. Navigate to the root directory of the project:

   ```
   cd /path/to/UnitApplication
   ```

2. Build the example using the provided build script:

   ```
   ./Build.sh --example=DS3231_RTC
   ```

3. The compiled binary files will be available in the output directory.

4. Flash the binary to your Pico using either the UF2 bootloader or the Flash_Application.sh script.

## Expected Output

When running, the example will:
1. Initialize I2C and verify the DS3231 is present on the bus
2. Set an initial date and time (Saturday, 22/03/2026, 09:30:00)
3. Continuously read and print the current date and time once per second

Example output:

```
DS3231 RTC Example
------------------

Initializing DS3231...
DS3231 initialized successfully.

Setting initial time: 09:30:00  Monday 22/03/2026
Time set successfully.

Reading time...
------------------
Monday  22/03/2026  09:30:00
Monday  22/03/2026  09:30:01
Monday  22/03/2026  09:30:02
Monday  22/03/2026  09:30:03
```

## Code Explanation

The example performs the following operations:

- Calls `DS3231_Init()`, which initialises the I2C bus and checks that the DS3231 acknowledges its address. It also reads the Status register and clears the **Oscillator Stop Flag (OSF)** if set. The OSF is set on the very first power-up and any time the oscillator was stopped (e.g. dead backup battery), indicating that the stored time cannot be trusted.
- Calls `DS3231_SetDateTime()` once to load a known date and time. In a real application this step would only be performed when necessary — for example when the OSF was detected on init, or when time is updated via a user command or network sync. The DS3231 continues counting independently on its backup battery once set.
- Enters a loop that calls `DS3231_GetDateTime()` every second and prints the result. All seven timekeeping registers are fetched in a single I2C burst read, so the snapshot is coherent.

## Day-of-Week Convention

The DS3231 stores a day-of-week value (1–7) but does not assign meaning to specific numbers — that mapping is user-defined, as long as values are sequential. This example uses **1 = Monday** through **7 = Sunday**, defined via the `day_names` array in `main.c`. Adjust the array and the initial `.day` value to match your preferred convention.

## Troubleshooting

1. **"DS3231 not found on I2C bus"**
   - Double-check SDA/SCL wiring and that the module is powered.
   - Confirm the I2C address is 0x68 (all address pins on the module tied to GND).
   - Verify that pull-up resistors are present on the I2C lines.

2. **Time resets to the set value every reboot**
   - The backup battery (CR2032) may be missing, flat, or not making contact. Without it, the DS3231 loses time when the Pico is unpowered.
   - In a production application, check the OSF flag after `DS3231_Init()` and only call `DS3231_SetDateTime()` if OSF was set, so previously stored time is preserved across reboots.

3. **Time drifts or is clearly wrong**
   - Ensure the crystal oscillator on the DS3231 module is not being disturbed by nearby noise sources.
   - The DS3231 is rated ±2ppm (0°C–40°C), which corresponds to roughly ±1 minute per year under normal conditions.
