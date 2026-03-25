#ifndef WIFI_OTA_DOWNLOAD_H
#define WIFI_OTA_DOWNLOAD_H

/**
 * OTA firmware download over HTTPS from an Apache2 server.
 *
 * Server configuration (IP, port, certificate): see WiFi_OTA_Config.h
 * Full Apache2 setup guide: see Docs/OTA_Apache2_Setup.md
 * Certificate generation script: see Tools/GenerateOTACert.sh
 */

/**
 * @brief Downloads firmware from the server over HTTPS.
 *
 * This function establishes a secure connection to the server,
 * downloads the firmware binary, and returns the number of bytes
 * downloaded. On error, it returns a negative value corresponding
 * to the Mbed TLS error code.
 *
 * @return Number of bytes downloaded on success, negative Mbed TLS error code on failure.
 */
int download_firmware(void);


#endif /* WIFI_OTA_DOWNLOAD_H */