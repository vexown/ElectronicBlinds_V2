#ifndef UBLOX_GPS_HAL_H
#define UBLOX_GPS_HAL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// GPS data structure to hold parsed information
typedef struct {
    // Position
    double latitude;      // Decimal degrees (negative for South)
    double longitude;     // Decimal degrees (negative for West)
    bool position_valid;  // Position validity flag
    
    // Time and Date
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    
    // Speed and Course
    float speed_knots;    // Speed in knots
    float course_deg;     // Course in degrees true
    
    // Altitude and quality
    float altitude;       // Altitude in meters above MSL
    uint8_t fix_quality;  // 0=no fix, 1=GPS, 2=DGPS
    uint8_t satellites_used;
    
    // Status
    bool valid;           // Overall data validity
} GPS_Data;

// Initialize the GPS hardware
void gps_init(void);

// Read a NMEA sentence from the GPS module
bool gps_read_nmea(char *buffer, size_t max_len, size_t *out_len);

// Parse a NMEA sentence and update the internal GPS data
bool gps_process_sentence(const char* sentence);

// Get the current GPS data
GPS_Data* gps_get_data(void);

#endif // UBLOX_GPS_HAL_H