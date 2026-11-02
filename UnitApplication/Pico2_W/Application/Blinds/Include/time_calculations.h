/**
 * @file time_calculations.h
 * @brief Implementation supporting sunrise/sunset, DST, and any other time-based calculations for the Blinds application.
 */

#ifndef TIME_CALCULATIONS_H
#define TIME_CALCULATIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*******************************************************************************/
/*                                   MACROS                                    */
/*******************************************************************************/
#define LATITUDE_SIEROSZEWICE    (51.63394) //in degrees
#define LONGITUDE_SIEROSZEWICE   (17.96725) //in degrees

/*
 * DS3231 TIME CONVENTION -- READ BEFORE SETTING THE RTC
 *
 * The DS3231 always holds Polish SUMMER time (CEST, UTC+2), all year round.
 * It is never adjusted for DST, neither by hand nor by the firmware.
 *
 *   - Setting it in summer: set it to the wall-clock time.
 *   - Setting it in winter: set it ONE HOUR AHEAD of the wall clock.
 *
 * The blinds schedule is computed directly in this UTC+2 frame and compared
 * against the raw RTC reading, so DST never enters the control logic.
 * Local (wall-clock) time is derived only for logging: see isDST().
 */
#define RTC_UTC_OFFSET_HOURS     (2)        //DS3231 holds CEST (UTC+2) year-round
#define STANDARD_UTC_OFFSET_HOURS (1)       //Polish standard (winter) time, CET (UTC+1)
#define DST_SWITCH_RTC_HOUR      (3U)       //EU switches at 01:00 UTC = 03:00 on the CEST RTC, both in March and October

#define BCD_TO_DEC 1
#define DEC_TO_BCD 0
#define INCORRECT_REQUEST (0xFF)
#define LOWER_NIBBLE_MASK (0x0F)


/*******************************************************************************/
/*                                 DATA TYPES                                  */
/*******************************************************************************/

/*******************************************************************************/
/*                                 PUBLIC API                                  */
/*******************************************************************************/

/**
 * @brief Converts degrees to radians using float for FPU acceleration.
 */
static inline float degToRad(float degrees) 
{
    return degrees * ((float)M_PI / 180.0f);
}

/**
 * @brief Converts radians to degrees using float for FPU acceleration.
 */
static inline float radToDeg(float radians) 
{
    return radians * (180.0f / (float)M_PI);
}

/**
 * @brief Calculates sunrise and sunset times based on the NOAA Solar Calculator.
 * This function uses Julian Day calculations and solar geometry to estimate
 * sunrise and sunset. Note that the result is returned in decimal hours.
 * @param latitude   The latitude of the observer in degrees (North is positive).
 * @param longitude  The longitude of the observer in degrees (East is positive).
 * @param dayOfYear  The day of the year (1-366).
 * @param timeZone   The UTC offset (e.g., Poland is +1 in winter, +2 in summer).
 * @param sunrise    [out] Pointer to store the calculated sunrise time in decimal hours.
 * @param sunset     [out] Pointer to store the calculated sunset time in decimal hours.
 * @note Reference: https://gml.noaa.gov/grad/solcalc/
 */
void CalculateSunriseSunset(float latitude, float longitude, int dayOfYear, int timeZone, float* sunrise, float* sunset);

/**
 * @brief Computes the cumulative day of the year (1 to 366).
 * Accounts for leap years and sums the days in the preceding months.
 * @param year   Year within the 2000s (0–99, e.g. 25 for 2025).
 * @param month  Month, 1–12.
 * @param day    Day of month, 1–31.
 * @return uint32_t The day number of the year.
 */
uint32_t CalculateDayOfYear(uint8_t year, uint8_t month, uint8_t day);

/**
 * @brief Determines if Daylight Saving Time (DST) is active at a given RTC time.
 * This implementation follows the European Union standard: DST starts on the
 * last Sunday of March and ends on the last Sunday of October, both at
 * 01:00 UTC. Because the RTC holds CEST (UTC+2) year-round, that instant is
 * 03:00 RTC time on both changeover days (DST_SWITCH_RTC_HOUR).
 * @param year   RTC year within the 2000s (0–99, e.g. 25 for 2025).
 * @param month  RTC month, 1–12.
 * @param day    RTC day of month, 1–31.
 * @param hour   RTC hour, 0–23 (CEST, see the DS3231 time convention above).
 * @return true  If DST is in effect at that moment.
 * @return false If standard time is in effect.
 */
bool isDST(uint8_t year, uint8_t month, uint8_t day, uint8_t hour);

/**
 * @brief Converts values between Decimal and Binary Coded Decimal (BCD).
 * @param valueToConvert The 16-bit value to be converted.
 * @param direction      The mode of conversion (DEC_TO_BCD or BCD_TO_DEC).
 * @return uint8_t       The converted value, or INCORRECT_REQUEST if the direction is invalid.
 */
uint8_t ConvertBCD(uint16_t valueToConvert, uint8_t direction);

#endif /* TIME_CALCULATIONS_H */