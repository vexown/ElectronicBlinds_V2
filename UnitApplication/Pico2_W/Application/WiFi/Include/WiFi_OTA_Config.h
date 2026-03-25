#ifndef WIFI_OTA_CONFIG_H
#define WIFI_OTA_CONFIG_H

/**
 * OTA Server Configuration
 *
 * This file contains all settings that must be updated when the OTA server
 * or network environment changes. When switching networks or regenerating
 * certificates, only this file needs to be modified.
 *
 * What to update:
 *   1. OTA_HTTPS_SERVER_IP_ADDRESS - IP of the machine running the Apache2 HTTPS server
 *   2. OTA_HTTPS_SERVER_PORT       - HTTPS port (usually 443)
 *   3. CA_cert_OTA_server_raw      - Self-signed CA certificate PEM for the server
 *
 * To regenerate the certificate and get the C string to paste here:
 *   ./Tools/GenerateOTACert.sh <server_ip>
 *
 * For full Apache2 server setup instructions:
 *   See Docs/OTA_Apache2_Setup.md
 */

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

#define OTA_HTTPS_SERVER_IP_ADDRESS  "192.168.50.194"  /* IP address of the Apache2 HTTPS server for OTA updates */
#define OTA_HTTPS_SERVER_PORT        443              /* HTTPS port for OTA server */
#define FIRMWARE_PATH                "/firmware.bin"   /* Path to firmware binary on the server */

/*******************************************************************************/
/*                              CERTIFICATE                                    */
/*******************************************************************************/

/**
 * Self-signed CA certificate for the Apache2 OTA server.
 * Only compiled into translation units that define OTA_INCLUDE_CERTIFICATE
 * before including this header (i.e. WiFi_OTA_download.c).
 *
 * Regenerate with: ./Tools/GenerateOTACert.sh <server_ip>
 * The script will output this string ready to paste here.
 *
 * The certificate CN must match OTA_HTTPS_SERVER_IP_ADDRESS above.
 */
#ifdef OTA_INCLUDE_CERTIFICATE
static const unsigned char *CA_cert_OTA_server_raw =
    (const unsigned char *)
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDYjCCAkqgAwIBAgIUaa4s3EhoHi7OW/10I1QpaRFmfXMwDQYJKoZIhvcNAQEL\n"
    "BQAwODEPMA0GA1UECgwGTW9kdXJpMQwwCgYDVQQLDANPVEExFzAVBgNVBAMMDjE5\n"
    "Mi4xNjguNTAuMTk0MB4XDTI2MDMyMTIyMzQ0NVoXDTI3MDMyMTIyMzQ0NVowODEP\n"
    "MA0GA1UECgwGTW9kdXJpMQwwCgYDVQQLDANPVEExFzAVBgNVBAMMDjE5Mi4xNjgu\n"
    "NTAuMTk0MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA34ZZxPcIvR2d\n"
    "1pU66o35jkUc2X07wVxefiShVv3FV9pK9wcH/11CK/2lJSN/5Ey0m+oqZgTzom6y\n"
    "3wJSUaH8DMATBR6ZbBhezSuSUcOtBIT+Z3W9UWDY5K/e9/iK2dc2xh5apulkqg/Q\n"
    "Wpo5CbDLUdhWZjjwSVbt+CaJQ4MPKFrFK1iXxuYdv8adFPGaVRAqeot6lLKwkoFy\n"
    "8l4wM96O/U0pYhcsJV8LpaJzgz7h2rTMzv6CZQTMvVMCQUxGzp5EGIDYWSO/w+uN\n"
    "qOmD/+m0tJWuqTGxyYyNEMmD/KgB5LyvAHFPIUb3G+nrhDuRS2MSFdOE7lD4WqGc\n"
    "WaV8es/1UwIDAQABo2QwYjAdBgNVHQ4EFgQUZoolK+l3XBn4WtG5iAYpSwAlxI4w\n"
    "HwYDVR0jBBgwFoAUZoolK+l3XBn4WtG5iAYpSwAlxI4wDwYDVR0TAQH/BAUwAwEB\n"
    "/zAPBgNVHREECDAGhwTAqDLCMA0GCSqGSIb3DQEBCwUAA4IBAQBMzbyuGNlkAtbq\n"
    "6Cp+Y/UBmrwMQ+joOBsEl7O9OewqbaMi852/bVZFh1sxWOPswT8ghqLpCNjqQ4vt\n"
    "H4gOhg9U86dq+c5mled1nrko1tFHOk8hLUVR+3/YPejsxAvRaTqSXjhmSSHRztNu\n"
    "tQIGuQPxTT57u5QK+qwA3zhYnMFYCKQDRbnpZcTEF73dkNjWmus36Fu7Lmnd5QdE\n"
    "IJCk8CR2vVUlEi/kW0QRqSdz3M+5e8hO0JWhIp4vWsfbaKqS1C1Q0Sh1o2rJbw5n\n"
    "eRwbGNH4bg0e9XyN5SxiVMJzwInNhDwNruwd36MJR8NIzL8vQKSz9wcvUKGmjlaQ\n"
    "dA8cdeaZ\n"
    "-----END CERTIFICATE-----\n";
#endif /* OTA_INCLUDE_CERTIFICATE */

#endif /* WIFI_OTA_CONFIG_H */
