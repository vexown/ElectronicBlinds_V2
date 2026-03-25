#include "UBLOX_GPS_HAL.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

static GPS_Data gps_data;
static char internal_buffer[256];
static size_t buf_len = 0;

static double nmea_to_decimal_degrees(const char* nmea_pos, char direction) {
    if (!nmea_pos || strlen(nmea_pos) == 0) return 0.0;
    double degrees = atof(nmea_pos) / 100.0;
    int deg = (int)degrees;
    double minutes = (degrees - deg) * 100.0;
    double decimal_degrees = deg + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') decimal_degrees = -decimal_degrees;
    return decimal_degrees;
}

static void parse_gsa(char* sentence, GPS_Data* data) {
    char* fields[18];
    int i = 0;
    fields[i] = strtok(sentence, ",");
    while (fields[i] && i < 17) fields[++i] = strtok(NULL, ",");
    
    if (i >= 17) {
        int fix_quality = atoi(fields[2]);
        data->fix_quality = (fix_quality > UINT8_MAX) ? UINT8_MAX : (uint8_t)fix_quality;
        data->valid = (data->fix_quality > 1);
        data->position_valid = data->valid;
        int sat_count = 0;
        for (int j = 3; j < 15; j++) {
            if (fields[j] && strlen(fields[j]) > 0) sat_count++;
        }
        int new_sat_count = data->satellites_used + sat_count;
        data->satellites_used = (new_sat_count > UINT8_MAX) ? UINT8_MAX : (uint8_t)new_sat_count;
    }
}

static void parse_gga(char* sentence, GPS_Data* data) {
    char* fields[15];
    int i = 0;
    fields[i] = strtok(sentence, ",");
    while (fields[i] && i < 14) fields[++i] = strtok(NULL, ",");
    
    if (i >= 9) {
        if (strlen(fields[1]) >= 6) sscanf(fields[1], "%2hhu%2hhu%2hhu", &data->hours, &data->minutes, &data->seconds);
        if (strlen(fields[2]) > 0) data->latitude = nmea_to_decimal_degrees(fields[2], fields[3][0]);
        if (strlen(fields[4]) > 0) data->longitude = nmea_to_decimal_degrees(fields[4], fields[5][0]);
        int fix_quality = atoi(fields[6]);
        data->fix_quality = (fix_quality > UINT8_MAX) ? UINT8_MAX : (uint8_t)fix_quality;
        data->valid = (data->fix_quality > 0);
        data->position_valid = data->valid;
        int satellites = atoi(fields[7]);
        data->satellites_used = (satellites > UINT8_MAX) ? UINT8_MAX : (uint8_t)satellites;
        if (strlen(fields[9]) > 0) data->altitude = (float)atof(fields[9]);
    }
}

static void parse_rmc(char* sentence, GPS_Data* data) {
    char* fields[13];
    int i = 0;
    fields[i] = strtok(sentence, ",");
    
    // Only keep the first tokenization loop, remove the second one
    while (fields[i] && i < 12) {
        fields[++i] = strtok(NULL, ",");
    }
    
    if (i >= 9) {  // We have at least up to the date field
        // Time parsing remains the same
        if (strlen(fields[1]) >= 6) {
            // Manual parsing to ensure correct time extraction
            char hour_str[3] = {fields[1][0], fields[1][1], '\0'};
            char min_str[3] = {fields[1][2], fields[1][3], '\0'};
            char sec_str[3] = {fields[1][4], fields[1][5], '\0'};
            
            data->hours = (uint8_t)atoi(hour_str);
            data->minutes = (uint8_t)atoi(min_str);
            data->seconds = (uint8_t)atoi(sec_str);
        }
        
        data->valid = (fields[2][0] == 'A');
        data->position_valid = data->valid;
        
        if (data->valid) {
            if (strlen(fields[3]) > 0) data->latitude = nmea_to_decimal_degrees(fields[3], fields[4][0]);
            if (strlen(fields[5]) > 0) data->longitude = nmea_to_decimal_degrees(fields[5], fields[6][0]);
            data->speed_knots = (float)atof(fields[7]);
            
            // Updated: Check if field 8 is a valid course (degrees) or date
            // If it contains periods it's likely a course, otherwise a date
            if (fields[8] && strlen(fields[8]) == 6) {
                // Extract individual date components manually to ensure correct parsing
                char day_str[3] = {fields[8][0], fields[8][1], '\0'};
                char month_str[3] = {fields[8][2], fields[8][3], '\0'};
                char year_str[3] = {fields[8][4], fields[8][5], '\0'};
                
                data->day = (uint8_t)atoi(day_str);
                data->month = (uint8_t)atoi(month_str);
                data->year = (uint16_t)(atoi(year_str) + 2000);  // 2-digit to 4-digit year
            }
            
            if (data->fix_quality == 0) data->fix_quality = 1;
        }
    }
}

static void parse_gsv(char* sentence, GPS_Data* data) {
    char* fields[20];
    int i = 0;
    fields[i] = strtok(sentence, ",");
    while (fields[i] && i < 19) fields[++i] = strtok(NULL, ",");
    
    // GSV format: $xxGSV,numMsg,msgNum,numSats,PRN,elev,azim,SNR,...*checksum
    if (i >= 3) {
        // Field 3 contains total number of satellites in view
        int satellites_in_view = atoi(fields[3]);
        
        // Only update if this is the first GSV message (msgNum == 1)
        if (fields[2] && atoi(fields[2]) == 1) {
            data->satellites_used = (satellites_in_view > UINT8_MAX) ? 
                UINT8_MAX : (uint8_t)satellites_in_view;
        }
    }
}

bool gps_parse_nmea(const char* nmea_sentence, GPS_Data* data) {
    char sentence[256];
    strncpy(sentence, nmea_sentence, sizeof(sentence) - 1);
    sentence[sizeof(sentence) - 1] = '\0';
    
    char* start = strchr(sentence, '$');
    if (!start) return false;
    
    char* end = strpbrk(start, "\r\n");
    if (end) *end = '\0';
    
    char type[6] = {0};
    strncpy(type, start + 1, 5);
    
    if (strstr(type, "RMC") != NULL) {
        // Don't reset satellites_used here
        parse_rmc(sentence, data);
        return true;
    } else if (strstr(type, "GGA") != NULL) { 
        parse_gga(sentence, data);
        return true;
    } else if (strstr(type, "GSA") != NULL) { 
        parse_gsa(sentence, data);
        return true;
    } else if (strstr(type, "GSV") != NULL) {
        // Add parsing for GSV messages
        parse_gsv(sentence, data);
        return true;
    }
    return false;
}

void gps_init(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

bool gps_read_nmea(char *buffer, size_t max_len, size_t *out_len) {
    while (true) {
        for (size_t i = 0; i < buf_len; i++) {
            if (internal_buffer[i] == '\n') {
                size_t sentence_len = i + 1;
                if (sentence_len > max_len) return false;
                memcpy(buffer, internal_buffer, sentence_len);
                *out_len = sentence_len;
                memmove(internal_buffer, internal_buffer + sentence_len, buf_len - sentence_len);
                buf_len -= sentence_len;
                return true;
            }
        }
        if (!uart_is_readable(UART_ID) || buf_len >= sizeof(internal_buffer) - 1) return false;
        char c = uart_getc(UART_ID);
        internal_buffer[buf_len++] = c;
    }
}

GPS_Data* gps_get_data(void) {
    return &gps_data;
}

bool gps_process_sentence(const char* sentence) {
    return gps_parse_nmea(sentence, &gps_data);
}