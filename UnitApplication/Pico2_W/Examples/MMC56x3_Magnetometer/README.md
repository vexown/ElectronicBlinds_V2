# MMC56x3 Magnetometer Example

This example demonstrates how to interface with the MMC56x3 magnetometer using the Raspberry Pi Pico. The code initializes the sensor over I2C and continuously reads magnetic field data.

## Hardware Requirements

- Raspberry Pi Pico or Pico W
- MMC56x3 magnetometer module
- Jumper wires
- Breadboard (optional)

## Connections

| Pico Pin | MMC56x3 Pin | Description |
|----------|-------------|-------------|
| GPIO 4   | SDA         | I2C Data    |
| GPIO 5   | SCL         | I2C Clock   |
| 3.3V     | VCC         | Power       |
| GND      | GND         | Ground      |

## Building the Example

1. Navigate to the root directory of the project:

   cd /path/to/UnitApplication

2. Build the example using the provided build script:

    ./Build.sh --example=MMC56x3_Magnetometer

3. The compiled binary files will be available in the output directory.

4. Flashing either via UF2 bootloader or using the Flash_Application.sh script

## Expected Output

When running, the example will:
1. Initialize I2C communication
2. Configure the MMC56x3 sensor
3. Output magnetic field readings (X, Y, Z) in µT
4. Also output temperature in °C
5. Update readings approximately 10 times per second

Example output:

```
MMC56x3 Magnetometer Example
----------------------------

Initializing I2C...
Creating MMC56x3 instance...
Initializing MMC56x3 sensor...
MMC56x3 initialized successfully!

Starting magnetic field readings...
Magnetic field: X=12.34, Y=-45.67, Z=78.90 µT
Temperature: 23.5 °C
Magnetic field: X=12.38, Y=-45.72, Z=78.85 µT
Temperature: 23.5 °C
```

## Code Explanation

The example performs the following operations:
- Initializes the Raspberry Pi Pico standard I/O for serial communication
- Sets up I2C communication on GPIO 4 (SDA) and GPIO 5 (SCL) at 400kHz
- Creates a magnetometer instance using the MMC56x3 wrapper
- Initializes the sensor and checks for successful initialization
- Performs a set/reset operation to improve accuracy
- Enters a continuous loop that:
    - Reads magnetic field data from the sensor
    - Displays X, Y, Z magnetic field values in microteslas (µT)
    - Shows the current temperature in degrees Celsius
    - Waits 100ms between readings (10Hz update rate)