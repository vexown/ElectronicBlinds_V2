Collecting workspace information# UBLOX GPS Example

This example demonstrates how to interface with a UBLOX GPS module using the Raspberry Pi Pico. The code initializes the GPS module over UART and continuously reads and parses NMEA sentences to extract location, time, and other GPS data.

## Hardware Requirements

- Raspberry Pi Pico 2 W
- UBLOX GPS module (NEO-6M, NEO-7M, NEO-8M, or compatible)
- Jumper wires
- Breadboard (optional)

## Connections

| Pico Pin | GPS Module Pin | Description |
|----------|--------------|-------------|
| GPIO 4   | TX           | UART TX     |
| GPIO 5   | RX           | UART RX     |
| 3.3V     | VCC          | Power       |
| GND      | GND          | Ground      |

## Building the Example

1. Navigate to the root directory of the project:

   ```
   cd /path/to/UnitApplication
   ```

2. Build the example using the provided build script:

   ```
   ./Build.sh --example=UBLOX_GPS
   ```

3. The compiled binary files will be available in the output directory.

4. Flash the binary to your Pico using either the UF2 bootloader or the Flash_Application.sh script.

## Expected Output

When running, the example will:
1. Initialize UART communication
2. Configure the GPS module
3. Continuously read and display NMEA sentences
4. Parse and display the extracted GPS data, including:
   - Time and date
   - Latitude and longitude
   - Altitude
   - Speed and course
   - Fix quality
   - Number of satellites used

Example output:

```
GPS Test Application
--------------------

Received NMEA sentence: $GNRMC,203629.00,A,5138.14679,N,01757.96556,E,0.013,,040325,5.90,E,D*3D
Parsed GPS Data:
  Time: 20:36:29
  Date: 04/03/2025
  Position: 51.635780° N, 17.966093° E
  Altitude: 0.0 m
  Speed: 0.013 knots
  Course: 0.0°
  Fix Quality: 1
  Satellites: 10
--------------------
```

## Code Explanation

The example performs the following operations:
- Initializes the Raspberry Pi Pico standard I/O for serial communication
- Sets up UART communication on GPIO 4 (TX) and GPIO 5 (RX) at 9600 baud
- Enters a continuous loop that:
  - Reads NMEA sentences from the GPS module
  - Displays the raw NMEA sentences
  - Processes each sentence to extract GPS data
  - Displays formatted GPS information when valid data is received
  - Updates every 10ms

## Troubleshooting Tips

1. **No NMEA sentences received**:
   - Ensure your GPS module has a clear view of the sky
   - Confirm connections are correct, especially TX/RX wires
   - Allow time for the GPS module to acquire a satellite fix (could take several minutes)

2. **Invalid position data**:
   - GPS modules need an unobstructed view of the sky to get a position fix
   - Indoor GPS reception is usually poor or non-existent
   - The first fix after powering on can take 1-5 minutes in optimal conditions

3. **No satellites detected**:
   - Check that your GPS module's antenna is properly connected
   - Move to a location with clearer sky visibility