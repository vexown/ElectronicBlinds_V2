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
#define TIME_ZONE_PLUS_TO_E      (2)

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
 * Converts BCD date components to decimal, accounts for leap years, and sums
 * the days in the preceding months.
 * @param yearBCD   The year in BCD format (e.g., 0x24 for 2024).
 * @param monthBCD  The month in BCD format (0x01 to 0x12).
 * @param dayBCD    The day in BCD format (0x01 to 0x31).
 * @return uint32_t The day number of the year.
 */
uint32_t CalculateDayOfYear(uint32_t yearBCD, uint32_t monthBCD, uint32_t dayBCD);

/**
 * @brief Determines if Daylight Saving Time (DST) is active for a given date.
 * This implementation follows the European Union standard: DST starts on the 
 * last Sunday of March and ends on the last Sunday of October.
 * @param yearBCD   The year in BCD format.
 * @param monthBCD  The month in BCD format.
 * @param dayBCD    The day in BCD format.
 * @return true     If the date falls within the DST period.
 * @return false    If the date is in standard time.
 */
bool isDST(uint32_t yearBCD, uint32_t monthBCD, uint32_t dayBCD);

/**
 * @brief Converts values between Decimal and Binary Coded Decimal (BCD).
 * @param valueToConvert The 16-bit value to be converted.
 * @param direction      The mode of conversion (DEC_TO_BCD or BCD_TO_DEC).
 * @return uint8_t       The converted value, or INCORRECT_REQUEST if the direction is invalid.
 */
uint8_t ConvertBCD(uint16_t valueToConvert, uint8_t direction);

#endif /* TIME_CALCULATIONS_H */