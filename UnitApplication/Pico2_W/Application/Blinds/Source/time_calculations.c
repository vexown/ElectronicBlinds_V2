
/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "time_calculations.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/
static uint8_t ZellersCongruence(int year, int month, int day);
static bool isLeapYear(uint32_t year);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

uint8_t ConvertBCD(uint16_t valueToConvert, uint8_t direction)
{
    uint8_t convertedValue;

    if(direction == DEC_TO_BCD)
    {
        convertedValue = (uint8_t)((((valueToConvert / 10) << 4) | (valueToConvert % 10)));
    }
    else if(direction == BCD_TO_DEC)
    {
        convertedValue = (uint8_t)((((valueToConvert >> 4) * 10) + (valueToConvert & LOWER_NIBBLE_MASK)));
    }
    else
    {
        convertedValue = INCORRECT_REQUEST;
    }

    return convertedValue;
}

bool isDST(uint32_t yearBCD, uint32_t monthBCD, uint32_t dayBCD)
{
    bool isTodayDST = false;

    uint32_t dayDEC = ConvertBCD((uint16_t)dayBCD, BCD_TO_DEC);
    uint32_t monthDEC = ConvertBCD((uint16_t)monthBCD, BCD_TO_DEC);
    uint32_t yearDEC = ConvertBCD((uint16_t)yearBCD, BCD_TO_DEC) + 2000; /* This code only works for 2nd millennium, should be enough lol */

    /* Check if DST is in effect (last Sunday in March to last Sunday in October) */
    if (monthDEC >= 3 && monthDEC <= 10) 
    {
        uint32_t lastSundayInMonth = 31; // Default to last day of the month
        // Check if the current month is March or October
        if (monthDEC == 3 || monthDEC == 10) 
        {
            // Calculate the last Sunday of the month
            while (ZellersCongruence((int)yearDEC, (int)monthDEC, (int)lastSundayInMonth) != 6) 
            {
                lastSundayInMonth--;
            }
            if((dayDEC >= lastSundayInMonth) && (monthDEC == 3)) /* If first, or any later, day of DST in March */
            {
                isTodayDST = true;
            }
            else if((dayDEC <= lastSundayInMonth) && (monthDEC == 10)) /* If last, or any previous, day of DST in October */
            {
                isTodayDST = true;
            }
            else /* If in March before DST starts (last Sunday) or if in October after DST ends (last Sunday) */
            {
                isTodayDST = false; 
            }
        }
        else /* Months from 4 to 9 */
        {
            isTodayDST = true;
        }
    } 
    else 
    {
        isTodayDST = false; // DST is not in effect outside the specified months
    }

    return isTodayDST;
}

uint32_t CalculateDayOfYear(uint32_t yearBCD, uint32_t monthBCD, uint32_t dayBCD) 
{
    uint32_t dayDEC = ConvertBCD((uint16_t)dayBCD, BCD_TO_DEC);
    uint32_t monthDEC = ConvertBCD((uint16_t)monthBCD, BCD_TO_DEC);
    uint32_t yearDEC = ConvertBCD((uint16_t)yearBCD, BCD_TO_DEC) + 2000; /* This code only works for 2nd millennium, should be enough lol */

    uint32_t daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    if (isLeapYear(yearDEC)) 
    {
        daysInMonth[2] = 29;
    }

    uint32_t currentDay = 0;
    for (uint32_t i = 1; i < monthDEC; i++) 
    {
        currentDay += daysInMonth[i];
    }

    /* Calculated currentDay: */
    currentDay += dayDEC; 

    return currentDay;
}

/* Use https://gml.noaa.gov/grad/solcalc/ and the excel doc at: https://gml.noaa.gov/grad/solcalc/calcdetails.html 
   for reference/calibration of your sunset/sunrise functions! */
/* The calculation will be accurate for DST time of the given time zone (Poland is E(east) + 2) */
/* Function to calculate the sunrise and sunset times based on NOAA solar calculator */
void CalculateSunriseSunset(float latitude, float longitude, int dayOfYear, int timeZone, float* sunrise, float* sunset) 
{
    // Define the time (12:00:00)
    float Time = 0.5f; 
    // Calculate the date
    int Date = 44927 + dayOfYear;
    // Calculate the Julian Day
    float JulianDay = (float)Date + 2415018.5f + Time - ((float)timeZone / 24.0f);
    // Calculate the Julian Century
    float JulianCentury = (JulianDay - 2451545.0f) / 36525.0f;
    // Calculate the eccentricity of Earth's orbit
    float EccentEarthOrbit = 0.016708634f - JulianCentury * (0.000042037f + 0.0000001267f * JulianCentury);
    // Calculate the geometric mean longitude of the Sun (in degrees)
    float GeomMeanLongSun = fmodf(280.46646f + JulianCentury * (36000.76983f + JulianCentury * 0.0003032f), 360.0f); 
    // Calculate the geometric mean anomaly of the Sun (in degrees)
    float GeomMeanAnomSun = 357.52911f + JulianCentury * (35999.05029f - 0.0001537f * JulianCentury); 
    // Calculate the mean obliquity of the ecliptic (in degrees)
    float MeanObliqEcliptic = 23.0f + (26.0f + ((21.448f - JulianCentury * (46.815f + JulianCentury * (0.00059f - JulianCentury * 0.001813f)))) / 60.0f) / 60.0f; 
    // Calculate the corrected obliquity (in degrees)
    float ObligCorr = MeanObliqEcliptic + 0.00256f * cosf(degToRad(125.04f - 1934.136f * JulianCentury)); 
    // Calculate the variable y
    float var_y = tanf(degToRad(ObligCorr / 2.0f)) * tanf(degToRad(ObligCorr / 2.0f));
    // Calculate the equation of time (in minutes)
    float EqOfTime = 4.0f * radToDeg(var_y * sinf(2.0f * degToRad(GeomMeanLongSun))
                - 2.0f * EccentEarthOrbit * sinf(degToRad(GeomMeanAnomSun))
                + 4.0f * EccentEarthOrbit * var_y * sinf(degToRad(GeomMeanAnomSun)) * cosf(2.0f * degToRad(GeomMeanLongSun))
                - 0.5f * var_y * var_y * sinf(4.0f * degToRad(GeomMeanLongSun))
                - 1.25f * EccentEarthOrbit * EccentEarthOrbit * sinf(2.0f * degToRad(GeomMeanAnomSun))); 

    // Calculate the solar noon (in LST)
    float SolarNoon = (720.0f - 4.0f * longitude - EqOfTime + ((float)timeZone * 60.0f)) / 1440.0f; 
    // Calculate the Sun's equation of center  
    float SunEqOfCtr = sinf(degToRad(GeomMeanAnomSun)) * (1.914602f - JulianCentury * (0.004817f + 0.000014f * JulianCentury))
                        + sinf(degToRad(2.0f * GeomMeanAnomSun)) * (0.019993f - 0.000101f * JulianCentury)
                        + sinf(degToRad(3.0f * GeomMeanAnomSun)) * 0.000289f;  
    // Calculate the Sun's true longitude
    float SunTrueLong = GeomMeanLongSun + SunEqOfCtr;
    // Calculate the Sun's apparent longitude (in degrees)
    float SunAppLong = SunTrueLong - 0.00569f - 0.00478f * sinf(degToRad(125.04f - 1934.136f * JulianCentury)); 
    // Calculate the Sun's declination
    float sunDeclin = radToDeg(asinf(sinf(degToRad(ObligCorr)) * sinf(degToRad(SunAppLong))));
    // Calculate the hour angle for sunrise (in degrees)
    float HA_Sunrise = radToDeg(acosf(cosf(degToRad(90.833f)) /
                        (cosf(degToRad(latitude)) * cosf(degToRad(sunDeclin)))
                        - tanf(degToRad(latitude)) * tanf(degToRad(sunDeclin)))); 

    // Calculate the sunrise time (in hours)
    *sunrise = ((SolarNoon * 1440.0f - HA_Sunrise * 4.0f) / 1440.0f) * 24.0f;
    // Calculate the sunset time (in hours)
    *sunset = ((SolarNoon * 1440.0f + HA_Sunrise * 4.0f) / 1440.0f) * 24.0f;
}


/*******************************************************************************/
/*                         STATIC FUNCTION DEFINITIONS                         */
/*******************************************************************************/


static uint8_t ZellersCongruence(int year, int month, int day) 
{
    if (month < 3) 
    {
        month += 12;
        year--;
    }

    int h = (day + ((13 * (month + 1)) / 5) + year + (year / 4) - (year / 100) + (year / 400)) % 7;
    
    /* Map the result to the corresponding day of the week (Monday = 0, ... , Sunday = 6) */
    return (uint8_t)((h + 5) % 7);
}

static bool isLeapYear(uint32_t year) 
{
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}