/**
 * File: WiFi_OTA_download.c
 * Description: Implementation of the firmware download process over WiFi using mbedTLS and our lwIP-based TCP stack.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* SDK includes */
#include "pico/cyw43_arch.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

/* MBEDTLS includes */
#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"

/* Project includes */
#include "flash_operations.h"
#include "flash_layout.h"
#include "WiFi_OTA_download.h"
#include "WiFi_TCP.h"
#include "WiFi_Common.h"
#define OTA_INCLUDE_CERTIFICATE
#include "WiFi_OTA_Config.h"
#include "Common.h"

/* Disable this module if OTA is not enabled */
#if (OTA_ENABLED == ON)

/**********************************************************************/
/*                          DEFINES                                   */
/**********************************************************************/
#define CHUNK_SIZE              65536       /* 32KB read chunks */
#define MAX_FIRMWARE_SIZE       1048576     /* 1MB maximum size */
#define HTTP_HEADER_MAX_SIZE    2048        /* Max HTTP header size */
#define HTTP_REQUEST_MAX_SIZE   512         /* Max HTTP request size */
#define HANDSHAKE_TIMEOUT_MS    15000       /* 15 seconds for TLS handshake */
#define READ_TIMEOUT_MS         30000       /* 30 seconds for reads */
#define RECONNECT_MAX_ATTEMPTS  3           /* Maximum reconnection attempts */

/**********************************************************************/
/*                   STATIC VARIABLE DECLARATIONS                     */
/**********************************************************************/
/* Expected size of the firmware to be downloaded.
 * This is set to 0 initially and will be updated once the HTTP headers are processed based on the Content-Length header. */
static size_t expected_size; 

/* Total number of bytes received during the download process. This is updated as data is received and processed. */
static size_t total_received;

/* Position in the flash buffer where the next chunk of data will be written. This is updated as data is written to flash. */
static size_t flash_buf_pos;

/* Timeout value for the TLS handshake and read operations. This is set to a specific duration (e.g., 15 seconds for handshake, 30 seconds for reads) */
static TickType_t timeout_value;

/* Flag to indicate whether the HTTP headers have been processed. This is used to determine if the Content-Length header has been read and parsed. */
static bool http_headers_processed;

/* Determines which bank we should download the new firmware to. We will always download to the inactive bank. */
static uint8_t inactive_bank;

/* Position in the HTTP header accumulation buffer. File-scope so download_firmware() can reset it
 * between calls - the HTTP headers for a single response may span multiple TCP reads, so the parser
 * needs to remember its position across decrypted chunks, but NOT across download attempts. */
static size_t header_buf_pos;

/* Number of reconnection attempts made during the current download. File-scope so download_firmware()
 * can reset it between calls. */
static int reconnect_attempts;

/**********************************************************************/
/*                  STATIC FUNCTION DECLARATIONS                      */
/**********************************************************************/

/**
 * Initializes the mbedTLS context, configuration, certificates, and RNG.
 * Sets up the SSL context for the OTA download connection.
 *
 * @param p_ssl_context Pointer to the mbedTLS SSL context structure.
 * @param p_ssl_config Pointer to the mbedTLS SSL configuration structure.
 * @param p_CA_cert_OTA_server Pointer to the mbedTLS X.509 certificate structure for the CA.
 * @param p_ctr_drbg_context Pointer to the mbedTLS CTR_DRBG context structure.
 * @param p_entropy_context Pointer to the mbedTLS entropy context structure.
 * @return 0 on success, -1 on failure.
 */
static int initialize_mbedtls_context(mbedtls_ssl_context *p_ssl_context,
                                      mbedtls_ssl_config *p_ssl_config,
                                      mbedtls_x509_crt *p_CA_cert_OTA_server,
                                      mbedtls_ctr_drbg_context *p_ctr_drbg_context,
                                      mbedtls_entropy_context *p_entropy_context);

/**
 * Processes the HTTP response headers from the server.
 * Extracts the content length and checks for errors in the response.
 *
 * @param p_decrypted_data_buffer Pointer to the decrypted data buffer.
 * @param p_decrypted_data_len Length of the decrypted data.
 * @param p_data_pos Pointer to the current position in the data buffer.
 * @return 0 if headers processed successfully, -1 on error.
 */
static int process_http_response_headers(const uint8_t *p_decrypted_data_buffer, size_t p_decrypted_data_len, 
                                         size_t* p_data_pos);

/**
 * Attempts to reconnect to the server if the connection is lost.
 * This function handles the reconnection process and re-sends the HTTP request.
 *
 * @param p_ssl_context Pointer to the mbedTLS SSL context structure.
 * @param http_request Pointer to the HTTP request string.
 * @return 0 on success, -1 on failure.
 */
static int attempt_reconnect(mbedtls_ssl_context *p_ssl_context, char *http_request);

/**
 * Processes the decrypted data received from the server.
 * This function handles the writing of data to flash and handles ssl read errors.
 *
 * @param p_decrypted_data_buffer Pointer to the decrypted data buffer.
 * @param p_ssl_read_status Status of the SSL read operation.
 * @param p_flash_buffer Pointer to the flash buffer for writing data.
 * @return 0 on success, -1 on error, 1 if more data is expected, -2 if reconnection is needed.
 */
static int process_decrypted_data(const uint8_t *p_decrypted_data_buffer, int p_ssl_read_status, uint8_t *p_flash_buffer);

/**
 * Writes the last chunk of data to flash memory.
 * This function handles the final write operation after all data has been received 
 * and there is still data remaining in the buffer waiting to be written to flash
 *
 * @param p_flash_buffer Pointer to the flash buffer containing the data to be written.
 * @param p_flash_buf_pos Position in the flash buffer where the next chunk of data will be written.
 * @return 0 on success, -1 on failure.
 */
static int write_last_chunk_to_flash(uint8_t *p_flash_buffer, size_t p_flash_buf_pos);

/**
 * Cleans up the mbedTLS context, configuration, and certificates.
 * Frees any allocated resources and resets the structures.
 *
 * @param p_ssl_context Pointer to the mbedTLS SSL context structure.
 * @param p_ssl_config Pointer to the mbedTLS SSL configuration structure.
 * @param p_CA_cert_OTA_server Pointer to the mbedTLS X.509 certificate structure for the CA.
 * @param p_ctr_drbg_context Pointer to the mbedTLS CTR_DRBG context structure.
 * @param p_entropy_context Pointer to the mbedTLS entropy context structure.
 */
static void clean_up(mbedtls_ssl_context *p_ssl_context, mbedtls_ssl_config *p_ssl_config,
                     mbedtls_x509_crt *p_CA_cert_OTA_server, mbedtls_ctr_drbg_context *p_ctr_drbg_context,
                     mbedtls_entropy_context *p_entropy_context);

/**********************************************************************/
/*                  GLOBAL FUNCTION DEFINITIONS                       */
/**********************************************************************/

int download_firmware(void) 
{
    /******************************* Variable declarations/definitions *******************************/
    /* Status variable */
    int result = -1;
    
    /* MbedTLS structures */
    /* SSL context is the main structure for an SSL/TLS connection,
     * storing the current session state, negotiated parameters, I/O buffers,
     * cryptographic keys, and connection settings. It maintains the entire 
     * lifecycle of an SSL/TLS connection, including handshake phases, record 
     * processing, and connection termination. This context is initialized with
     * mbedtls_ssl_init() and configured via mbedtls_ssl_setup() before use. */
    mbedtls_ssl_context ssl_context;

    /* SSL configuration that holds settings for the TLS connection including security
     * parameters, certificate verification options, and allowed cipher suites. This serves
     * as a template of settings applied to the ssl_context during setup. It allows
     * configuring the TLS/SSL connection behavior before the handshake begins. */
    mbedtls_ssl_config ssl_config;

    /* X.509 certificate structure that stores and manages the parsed CA (Certificate Authority)
     * certificate used to verify the server's identity. For TLS clients, this holds the trusted
     * root certificates used to validate the server's certificate, establishing a chain of trust. */
    mbedtls_x509_crt CA_cert_OTA_server;

    /* CTR_DRBG (Counter-mode Deterministic Random Bit Generator) is a NIST-standardized 
     * cryptographically secure pseudorandom number generator that uses a block cipher
     * in counter mode operation. As defined in NIST SP 800-90A, it provides strong
     * cryptographic randomness for security-critical applications. Here, it generates
     * random values for the TLS handshake and secure communications. The implementation
     * uses AES as the underlying block cipher. */
    mbedtls_ctr_drbg_context ctr_drbg_context;

    /* Entropy context that accumulates randomness from multiple sources (hardware or system)
     * to provide seed material for the DRBG. This structure is critical for initializing
     * cryptographically secure random number generation by collecting true entropy from
     * the system, which is then used to seed the deterministic random bit generator. */
    mbedtls_entropy_context entropy_context;

    /* Flash variables */
    static uint8_t flash_buffer[CHUNK_SIZE];

    /******************************* Initialization *******************************/

    /* Reset module-level state. These file-scope statics persist across calls;
     * without resetting them, a second download_firmware() invocation in the same
     * boot would inherit stale values (e.g. total_received == expected_size would
     * immediately satisfy the "download complete" condition). */
    expected_size = 0;
    total_received = 0;
    flash_buf_pos = 0;
    http_headers_processed = false;
    header_buf_pos = 0;
    reconnect_attempts = 0;

    LOG("=== Starting firmware download ===\n");
    LOG("Target server: https://%s%s\n", OTA_HTTPS_SERVER_IP_ADDRESS, FIRMWARE_PATH);

    watchdog_disable(); // Disable watchdog during OTA process

    uint8_t active_bank = check_active_bank();
    if (active_bank == INVALID_BANK) 
    {
        LOG("Failed to read active bank from flash\n");
        return result;
    }
    else
    {
        inactive_bank = (active_bank == BANK_A) ? BANK_B : BANK_A; // Set the inactive bank to the opposite of the active one
    }
    
    /* Initialize mbedTLS */
    if (initialize_mbedtls_context(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context) != 0)
    {
        LOG("Failed to initialize mbedTLS context\n");
        clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
        return result;
    }

    /*************************** Establishing Connection **************************/

    /* Perform the TLS handshake. 
     * This process might require multiple steps when using non-blocking I/O, as configured with mbedtls_ssl_set_bio.
     * The mbedtls_ssl_handshake function will return specific error codes MBEDTLS_ERR_SSL_WANT_READ or MBEDTLS_ERR_SSL_WANT_WRITE if it needs
     * to wait for network data to be received or sent, respectively. This loop handles these non-fatal "want" conditions by calling the function again
     * after short delay, allowing other tasks (like the network stack) to run and potentially satisfy the read/write requirement.
     * The loop continues until the handshake completes successfully (handshake_result == 0), a fatal error occurs (handshake_result is neither 0, 
     * WANT_READ, nor WANT_WRITE) or the timeout expires. */
    LOG("Performing TLS handshake...\n");
    timeout_value = xTaskGetTickCount() + pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS);
    int handshake_result = mbedtls_ssl_handshake(&ssl_context);
    while (handshake_result != 0) 
    {
        /* Check for fatal errors, non-fatal "want" conditions or timeout */
        if ((handshake_result != MBEDTLS_ERR_SSL_WANT_READ) && (handshake_result != MBEDTLS_ERR_SSL_WANT_WRITE)) 
        {
            LOG("TLS handshake failed: %08x\n", -handshake_result);
            clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
            return result;
        }
        else if (xTaskGetTickCount() >= timeout_value)
        {
            LOG("TLS handshake timeout\n");
            clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
            return result;
        }
        else
        {
            /* Continue the handshake process */
            handshake_result = mbedtls_ssl_handshake(&ssl_context);
        }
        
        /* Give other tasks a chance to run while we wait for handshake data to be available */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    LOG("TLS handshake completed successfully\n");

    /*************************** Requesting the firmware from the server **************************/

    /* Craft the HTTP GET request to download the firmware. It follows a standard HTTP/1.1 format */
    static char http_request[HTTP_REQUEST_MAX_SIZE];
    snprintf(http_request, sizeof(http_request), 
            "GET %s HTTP/1.1\r\n"               // Specifies the GET method, the resource path, and the HTTP version.
            "Host: %s\r\n"                      // Specifies the host header with the server IP address.
            "Connection: keep-alive\r\n"        // Indicates that the TCP connection should be kept alive after the response.
            "Cache-Control: no-cache\r\n"       // Instructs the server to not use cached data (always get the latest version).
            "User-Agent: Pico-W-OTA/1.0\r\n"    // Identifies the client making the request (can be any string).
            "\r\n",                             // End of HTTP headers (\r\n\r\n)
            FIRMWARE_PATH, OTA_HTTPS_SERVER_IP_ADDRESS);
    
    /* Send the composed HTTP GET request to the server through the established secure TLS connection.
     * mbedtls_ssl_write handles the encryption of the request data before sending it over the underlying TCP socket.
     * It returns the number of bytes successfully written, or a negative error code.
     * 
     * Note: In non-blocking mode, this could also return MBEDTLS_ERR_SSL_WANT_WRITE, indicating the underlying network layer is not ready to send yet.
     *       However, this specific implementation doesn't explicitly handle WANT_WRITE here, assuming the send buffer has enough space or blocking slightly. */
    LOG("Sending HTTP request...\n");
    if (mbedtls_ssl_write(&ssl_context, (const unsigned char *)http_request, strlen(http_request)) <= 0) 
    {
        LOG("Failed to send HTTP http_request\n");
        clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
        return result;
    }

    /****************************** Download Process ******************************/

    timeout_value = xTaskGetTickCount() + pdMS_TO_TICKS(READ_TIMEOUT_MS); // Allow some time for each read operation (resets after each successful read)
    int ssl_read_status;
    while (xTaskGetTickCount() < timeout_value) 
    {
        /**
         * Attempt to read and decrypt application data received from the peer (OTA server) via the established secure TLS session (`ssl_context`).
         * 
         * How it works internally:
         * 1. Checks if there's already decrypted application data buffered within the `ssl_context`. 
         *    If yes, it copies up to `sizeof(decrypted_data_buffer)` bytes into `decrypted_data_buffer` and returns the amount copied.
         *    (Internal input mbedTLS buffer has size `MBEDTLS_SSL_IN_BUFFER_SIZE` - typically around 16KB, same as output buffer).
         * 2. If no decrypted data is buffered, it needs to read encrypted data from the underlying network transport. It does this by calling 
         *    the receive callback function that was registered using `mbedtls_ssl_set_bio` (in our case, `tcp_receive_mbedtls_callback`).
         * 3. The `tcp_receive_mbedtls_callback` attempts to read data from the TCP socket using our lwIP-based TCP stack.
         * 4. If the callback returns encrypted data, `mbedtls_ssl_read` processes the TLS record(s): it decrypts the payload and verifies its integrity.
         * 5. If the decrypted data is application data (like the HTTP response body), it's copied to the provided `decrypted_data_buffer`.
         * 6. If the received TLS record is a control message (e.g., a re-handshake request or an alert), mbedTLS handles it internally. 
         *    `mbedtls_ssl_read` might then return a non-fatal error like `MBEDTLS_ERR_SSL_WANT_READ` to indicate that the application should 
         *    call it again after the internal processing is complete or more data is received.
         * 
         * Non-Blocking Operation:
         * Since we configured the BIO callbacks for non-blocking operation, `mbedtls_ssl_read` (and the underlying `tcp_receive_mbedtls_callback`) 
         * might not be able to read data immediately if none is available on the socket. In this scenario:
         * - `tcp_receive_mbedtls_callback` returns `MBEDTLS_ERR_SSL_WANT_READ`.
         * - `mbedtls_ssl_read` propagates this return value back to the caller.
         * This is not a fatal error; it simply means the application should retry the read operation later, allowing the network stack time to 
         * receive data. The `vTaskDelay` in the loop handles this. Similarly, `MBEDTLS_ERR_SSL_WANT_WRITE` could be returned if processing the 
         * received data requires sending a response (e.g., a TLS alert), but the send buffer is full. */
        uint8_t decrypted_data_buffer[1024]; // holds data received from the server and decrypted by the TLS layer
        ssl_read_status = mbedtls_ssl_read(&ssl_context, decrypted_data_buffer, sizeof(decrypted_data_buffer));

        /* Process the decrypted data (e.g., log it, check for errors, etc.) */
        int processing_status = process_decrypted_data(decrypted_data_buffer, ssl_read_status, flash_buffer);
        if(processing_status == 0)
        {
            LOG("All decrypted data processed successfully\n");
            break; // Exit the loop if all data has been processed
        }
        else if(processing_status == 1)
        {
            LOG("Decrypted data processed, but more data expected\n");
            continue; // Continue reading if more data is expected
        }
        else if(processing_status == -2)
        {
            if(attempt_reconnect(&ssl_context, http_request) == 0)
            {
                LOG("Reconnected successfully, resuming download...\n");
                continue; // Retry reading after successful reconnection
            }
            else
            {
                LOG("Failed to reconnect, aborting download\n");
                clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
                return result; // Exit if reconnection fails
            }
        }
        else /* Negative values indicate an error (this also covers unexpected positive values but should never happen) */
        {
            LOG("Failed to process decrypted data\n");
            clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
            return result;
        }
    }
    if (xTaskGetTickCount() >= timeout_value)
    {
        LOG("Firmware download timed out after %d ms\n", READ_TIMEOUT_MS);
        clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
        return result;
    }
    /* If we reach here, the loop exited because processing_status == 0 (download complete) */
    
    /* All data has been received and processed. Now write the last chunk to flash memory */
    if(write_last_chunk_to_flash(flash_buffer, flash_buf_pos) == 0)
    {
        LOG("Last chunk written to flash successfully\n");
    }
    else
    {
        LOG("Failed to write last chunk to flash\n");
        clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);
        return result;
    }

    /* Check if we received all the expected data */
    if (total_received == expected_size) 
    {
        LOG("Firmware download completed successfully: %zu bytes\n", total_received);
        result = 0;  // Success
    } 
    else if (total_received > 0) 
    {
        LOG("Firmware download partial: %zu/%d bytes\n", total_received, expected_size);
        result = (int)total_received;  // Partial success
    } 
    else 
    {
        LOG("Firmware download failed\n");
        /* result already initialized to -1 */
    }
    
    /*************************** Finalization and clean up **************************/
    clean_up(&ssl_context, &ssl_config, &CA_cert_OTA_server, &ctr_drbg_context, &entropy_context);

    LOG("=== Firmware download complete ===\n");

    return result;
}


/**********************************************************************/
/*                  STATIC FUNCTION DEFINITIONS                       */
/**********************************************************************/

static int initialize_mbedtls_context(mbedtls_ssl_context *p_ssl_context,
                                      mbedtls_ssl_config *p_ssl_config,
                                      mbedtls_x509_crt *p_CA_cert_OTA_server,
                                      mbedtls_ctr_drbg_context *p_ctr_drbg_context,
                                      mbedtls_entropy_context *p_entropy_context)
{
    /* Initialize mbedTLS structures to a clean, defined state */
    mbedtls_ssl_init(p_ssl_context);
    mbedtls_ssl_config_init(p_ssl_config);
    mbedtls_x509_crt_init(p_CA_cert_OTA_server);
    mbedtls_ctr_drbg_init(p_ctr_drbg_context);
    mbedtls_entropy_init(p_entropy_context);

    /* Seed the CTR_DRBG (Counter-mode Deterministic Random Bit Generator).
     * This function initializes the internal state of the DRBG using entropy
     * gathered from the system, making its output cryptographically secure.
     *
     * Explanation of the crucial parameters:
     * - mbedtls_entropy_func: The callback function used to retrieve entropy
     *                         from the registered sources within the entropy_context.
     * - p_entropy_context: The entropy context holding the accumulated randomness
     *                     sources (e.g., hardware RNG, timing variations).
     * - "pico_w_ota": A personalization string. This is application-specific data
     *                 mixed with the entropy during seeding. Its purpose, as defined
     *                 in NIST SP 800-90A, is to make this specific DRBG instance unique.
     *                 Using "pico_w_ota" helps differentiate this RNG instance used for
     *                 the OTA download process from any other potential RNG usage on
     *                 the device.
     *
     * A successful seeding is critical for generating unpredictable random numbers
     * required for the TLS handshake (e.g., nonces, key generation). */
    LOG("Seeding random number generator...\n");
    const unsigned char *personalization_string = (const unsigned char *)"pico_w_ota";
    const size_t personalization_string_len = strlen((const char *)personalization_string);
    if(mbedtls_ctr_drbg_seed(p_ctr_drbg_context, mbedtls_entropy_func, p_entropy_context, personalization_string, personalization_string_len) != 0) 
    {
        LOG("Failed to seed RNG\n");
        return -1;
    }

    /**
     * Parse the CA certificate for the OTA server.
     * 
     * This function decodes one or more X.509 certificates from a given buffer (CA_cert_OTA_server_raw) and populates an `mbedtls_x509_crt` 
     * (p_CA_cert_OTA_server) structure. Input buffer can contain a cert in either PEM (Privacy-Enhanced Mail, typically Base64 encoded 
     * with -----BEGIN/END----- markers) or DER (Distinguished Encoding Rules, usually a hex string) format. The function auto detects the format.
     * 
     * p_CA_cert_OTA_server structure with the parsed cert will later be passed to `mbedtls_ssl_conf_ca_chain` to configure the
     * SSL/TLS context (`p_ssl_config`). This tells the TLS client (our Pico W) which CAs it should trust when verifying the certificate 
     * presented by the OTA server during the TLS handshake. This step is essential for authenticating the server and ensuring
     * we are connecting to the legitimate OTA server, preventing man-in-the-middle attacks. */
    LOG("Loading CA certificate...\n");
    size_t CA_cert_OTA_server_raw_len = strlen((const char *)CA_cert_OTA_server_raw) + 1; // +1 for null terminator
    if (mbedtls_x509_crt_parse(p_CA_cert_OTA_server, CA_cert_OTA_server_raw, CA_cert_OTA_server_raw_len) != 0) 
    {
        LOG("Failed to parse CA certificate\n");
        return -1;
    }
    
    /**
     * Load default SSL/TLS configuration values into the p_ssl_config structure.
     * 
     * This function initializes the `mbedtls_ssl_config` structure (`p_ssl_config`) with a set of standard default values appropriate 
     * for the specified role (client/server), transport type (stream/datagram), and a chosen preset profile. These defaults typically 
     * include enabling secure protocol versions (like TLS 1.2 or higher), selecting a list of strong cipher suites, and setting reasonable 
     * buffer sizes. It serves as a convenient starting point, ensuring the configuration is in a known, sensible state before applying 
     * specific customizations. */
    LOG("Setting up TLS configuration...\n");
    if(mbedtls_ssl_config_defaults(p_ssl_config, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT) != 0) 
    {
        LOG("Failed to set TLS configuration defaults\n");
        return -1;
    }

    /**
     * Set the certificate verification mode for the SSL/TLS configuration.
     * 
     * This function configures how the peer's certificate chain should be verified during the TLS handshake. It determines the 
     * level of authentication required.
     * 
     * Authentication modes:
     *              - `MBEDTLS_SSL_VERIFY_NONE`: Peer certificate is not checked (Insecure, vulnerable to MITM attacks).
     *              - `MBEDTLS_SSL_VERIFY_OPTIONAL`: Peer certificate is checked if presented, but the handshake continues even 
     *                 if verification fails or no certificate is sent. The verification result is available via `mbedtls_ssl_get_verify_result`.
     *              - `MBEDTLS_SSL_VERIFY_REQUIRED`: Peer must present a certificate, and it must be validly verified against the configured 
     *                 trusted CAs. Handshake fails if verification fails. (Most common for clients). */
    mbedtls_ssl_conf_authmode(p_ssl_config, MBEDTLS_SSL_VERIFY_OPTIONAL);

    /**
     * Set the trusted Certificate Authority (CA) chain for certificate verification.
     * 
     * This function provides the SSL/TLS configuration with the set of CA certificates that should be considered trustworthy for verifying 
     * the peer's certificate chain.
     * 
     * p_CA_cert_OTA_server is the parsed CA certificate structure that contains the trusted root CA certificate(s) used to validate the server's
     * certificate during the TLS handshake. This is crucial for establishing a secure connection and ensuring that the server's identity is
     * authentic.
     * 
     * Last parameter ca_crl is a pointer to an `mbedtls_x509_crl` structure that contains Certificate Revocation Lists (CRLs) associated with the CAs.
     * It can be `NULL` if CRL checking is not required or not available. */
    mbedtls_ssl_conf_ca_chain(p_ssl_config, p_CA_cert_OTA_server, NULL);

    /**
     * Set the random number generator (RNG) callback function for the SSL/TLS configuration.
     * 
     * This function, `mbedtls_ssl_conf_rng`, configures the source of randomness that the SSL/TLS stack will use for cryptographic operations, 
     * such as generating session keys, nonces, and other security-critical values during the handshake and subsequent communication.
     * 
     * f_rng argument is a pointer to the RNG function. This function will be called by mbedTLS whenever it needs random bytes. 
     * We use `mbedtls_ctr_drbg_random`, which is the standard function to get random bytes from a CTR-DRBG context. */
    mbedtls_ssl_conf_rng(p_ssl_config, mbedtls_ctr_drbg_random, p_ctr_drbg_context);

    /**
     * Apply the configured settings to the SSL context.
     * 
     * This function takes the settings defined in the `mbedtls_ssl_config` structure (`p_ssl_config`) and applies them to the 
     * `mbedtls_ssl_context` structure (`p_ssl_context`). This effectively prepares the SSL context for initiating a connection using 
     * the specified configuration, including the trusted CA certificates, authentication mode, RNG functions, etc. */
    if(mbedtls_ssl_setup(p_ssl_context, p_ssl_config) != 0) 
    {
        LOG("Failed to apply SSL configuration\n");
        return -1;
    }

    /**
     * Set the hostname to check against the received server certificate. 
     * 
     * It sets the ServerName TLS extension too, if that extension is enabled. (client-side only)
     * 
     * This function informs the SSL context about the hostname (or IP address string) of the server it intends to connect to. 
     * This serves two primary purposes:
     * 
     * 1. Server Name Indication (SNI): During the TLS handshake (ClientHello message), the client sends this hostname to the server. 
     *    This allows servers hosting multiple secure websites on a single IP address to identify which site the client wants 
     *    and present the corresponding correct certificate.
     * 
     * 2. Hostname Verification: After receiving the server's certificate, the TLS client (if verification is enabled via 
     *    `mbedtls_ssl_conf_authmode`) checks if the hostname set here matches the identity presented in the certificate 
     *    (usually in the Subject Alternative Name (SAN) extension or, as a fallback, the Common Name (CN) field).
     *    This is a crucial security step to prevent Man-in-the-Middle (MitM) attacks, ensuring that the certificate is valid 
     *    not just cryptographically, but specifically for the server we intended to connect to. */
    if(mbedtls_ssl_set_hostname(p_ssl_context, OTA_HTTPS_SERVER_IP_ADDRESS) != 0) 
    {
        LOG("Failed to set hostname\n");
        return -1; // Cleanup will be handled by the caller
    }

    /**
     * Configure the SSL I/O callbacks for TLS communication
     * 
     * This function links the SSL/TLS layer with our TCP client's network functions.
     * It sets up how encrypted data will be sent and received over the network:
     * - p_ssl_context: The SSL context that will use these callbacks
     * - clientGlobal: Connection context passed to callbacks (contains socket info)
     * - tcp_send_mbedtls_callback: Function called when SSL needs to send data
     * - tcp_receive_mbedtls_callback: Function called when SSL needs to receive data
     * - NULL: blocking read callback with timeout, not used here (non-blocking)
     * 
     * This is a critical part of the TLS setup that allows mbedTLS to handle the 
     * encryption/decryption while delegating actual network I/O to our TCP layer.
     */
    mbedtls_ssl_set_bio(p_ssl_context, clientGlobal, 
                        (mbedtls_ssl_send_t*)tcp_send_mbedtls_callback,
                        (mbedtls_ssl_recv_t*)tcp_receive_mbedtls_callback,
                        NULL);

    LOG("mbedTLS context initialized successfully.\n");
    return 0; // Success
}


static int process_http_response_headers(const uint8_t *pp_decrypted_data_buffer, size_t p_decrypted_data_len, size_t* p_data_pos) 
{
    /* We use a separate temporary buffer to store the HTTP headers as we parse them.
     * We do it mainly because we want to null-terminate the string for easier processing (using strncmp, strstr, etc.)
     * It is a bit of a waste of memory, but we don't expect the headers to be very large. If needed we can change this process in the future. */
    static uint8_t http_header_buffer[HTTP_HEADER_MAX_SIZE];

    /* Note: header_buf_pos is now file-scope (see top of file) so that download_firmware()
     * can reset it between invocations. */

    /* Length of the content based on the Content-Length header */
    size_t content_length = 0;

    /* Variables to check for HTTP response codes */
    bool is_200_ok = false;
    bool is_206_partial = false;

    /* Go through the decrypted data buffer and look for the HTTP headers */
    while ((*p_data_pos < p_decrypted_data_len) && (header_buf_pos < HTTP_HEADER_MAX_SIZE)) 
    {
        http_header_buffer[header_buf_pos++] = pp_decrypted_data_buffer[(*p_data_pos)++]; // go byte by byte
        
        /* Scan through an HTTP response buffer to locate the boundary between HTTP headers and the message body. 
        * In HTTP/1.1, headers are separated from the body by a specific sequence: CRLF CRLF (carriage return + line feed twice) (\r\n\r\n).
        * Check each position in the buffer for the header termination sequence
        * We need at least 4 bytes to check for \r\n\r\n, so stop 3 bytes before the end */
        if (header_buf_pos >= 4 &&
            http_header_buffer[header_buf_pos - 4] == '\r' && 
            http_header_buffer[header_buf_pos - 3] == '\n' && 
            http_header_buffer[header_buf_pos - 2] == '\r' && 
            http_header_buffer[header_buf_pos - 1] == '\n') 
        {
            /* Null-terminate the header buffer (for the string functions) */
            http_header_buffer[header_buf_pos] = 0;

            /* Check for expected HTTP status codes: 200 OK (initial request) or 206 Partial Content (resume request) */
            is_200_ok = (strncmp((char*)http_header_buffer, "HTTP/1.1 200", 12) == 0);
            is_206_partial = (strncmp((char*)http_header_buffer, "HTTP/1.1 206", 12) == 0);
            
            if (is_200_ok) 
            {
                /* 200 OK: This indicates that the server has successfully processed the request and is sending the firmware data. */
                LOG("HTTP response code: 200 OK\n");
            }
            else if(is_206_partial) 
            {
                /* 206 Partial Content: This indicates that the server is sending a partial response in response to a range request.
                    * We need to handle such case due to the possibility of resuming downloads (see attempt_reconnect function). */
                LOG("HTTP response code: 206 Partial Content\n");
            }
            else
            {
                LOG("Unexpected HTTP response code. Header: %.*s\n", 32, http_header_buffer); // print max 32 character from the header
                header_buf_pos = 0; // Reset buffer position on error
                return -1;
            }
            
            /* Parse the Content-Length header to get the size of the firmware data.
                * This is important to know how much data we need to receive and write to flash.
                * The Content-Length header specifies the size of the body in bytes. */
            char *content_length_string = strstr((char*)http_header_buffer, "Content-Length:");
            if (content_length_string) 
            {
                /* Skip the "Content-Length: " part to get the actual number (+15 to skip the header name and space)
                    * and then convert it to an unsigned long integer using strtoul (base 10) */
                content_length = strtoul(content_length_string + 15, NULL, 10);

                /* What we do with the content length depends on the HTTP response code */
                if (is_200_ok) 
                {
                    if (content_length > MAX_FIRMWARE_SIZE) 
                    {
                        LOG("Error: Content-Length (%zu) exceeds maximum firmware size (%d)\n", content_length, MAX_FIRMWARE_SIZE);
                        header_buf_pos = 0; // Reset buffer position on error
                        return -1;
                    }
                    else if (content_length == 0)
                    {
                        LOG("Error: Content-Length is zero in 200 OK response\n");
                        header_buf_pos = 0; // Reset buffer position on error
                        return -1;
                    }
                    else /* Firmware size is within expected range */                        
                    {
                        expected_size = content_length; // Only set the global expected_size on the initial 200 OK response
                        LOG("Firmware total size from Content-Length: %zu bytes\n", expected_size);
                    }
                } 
                else if (is_206_partial) /* Here, we don't modify expected_size, as it should already be set. */                
                {
                    /* For 206, content_length is the size of the partial chunk */
                    LOG("Partial content length: %zu bytes\n", content_length);
                    if (content_length == 0) 
                    {
                        LOG("Warning: Content-Length is zero in 206 Partial Content response\n");
                        // This might be valid if the requested range was empty, but could indicate an issue. For now, we just log it.
                    }
                    /* Check for Content-Range header, which is mandatory for 206 */
                    char *content_range_string = strstr((char*)http_header_buffer, "Content-Range:");
                    if (!content_range_string) 
                    {
                        LOG("Error: Content-Range header missing in 206 Partial Content response\n");
                        header_buf_pos = 0; // Reset buffer position on error
                        return -1;
                    }
                    // Optional: Parse Content-Range for verification (e.g., check if it matches total_received)
                    // Example: "Content-Range: bytes 500-999/1000" (TODO, for now we just log it)
                    LOG("Content-Range header found: %.*s\n", 64, content_range_string); 
                }
                /* 'else' case will never happen because we return -1 on any response code other than 200 or 206 */
            } 
            else 
            {
                LOG("Error: Content-Length header not found\n");
                header_buf_pos = 0; // Reset buffer position on error
                return -1;
            }

            LOG("HTTP headers processed successfully\n");
            header_buf_pos = 0; // Reset header buffer position for potential future use (though unlikely in current flow)
            return 0; // Indicate headers processed successfully
        }
    }

    /* If the loop finished because the header buffer is full but no \r\n\r\n was found */
    if (header_buf_pos >= HTTP_HEADER_MAX_SIZE) 
    {
        LOG("Error: HTTP header exceeded max size (%d bytes)\n", HTTP_HEADER_MAX_SIZE);
        header_buf_pos = 0; // Reset buffer position
        return -1;
    }
    
    /* If we exit here, it means we processed some data but haven't found the end of headers yet.
     * We return 1 to indicate that more data is needed for the headers. */
    return 1; 
}

static int process_decrypted_data(const uint8_t *p_decrypted_data_buffer, int p_ssl_read_status, uint8_t *p_flash_buffer)
{
    if (p_ssl_read_status > 0) // Successfully read and decrypted data
    {
        size_t decrypted_data_len = (size_t)p_ssl_read_status; // Rename for clarity. Cast is safe since p_ssl_read_status is positive
        LOG("Received %zu bytes of decrypted data\n", decrypted_data_len);

        timeout_value = xTaskGetTickCount() + pdMS_TO_TICKS(READ_TIMEOUT_MS); // Reset timeout on successful read
        
        /* Position within the decrypted data buffer */
        size_t decrypted_data_pos = 0;
        
        /* Process the HTTP response to our HTTP GET request. 
         * The first part of the response contains HTTP headers, which we need to parse to get the content length.
         * The second part contains the actual firmware data. */
        if (!http_headers_processed) 
        {
            int header_status = process_http_response_headers(p_decrypted_data_buffer, decrypted_data_len, &decrypted_data_pos);
            
            if (header_status == 0) // Headers processed successfully
            {
                http_headers_processed = true; // set a flag so we can proceed to process the body of the response
            }
            else if (header_status == 1) // More data needed for headers
            {
                /* Headers are not fully processed yet. Return 1 to signal the main loop to read more data from the socket. Don't process body data yet. */
                return 1; // Indicate more data needed for headers
            }
            else // header_status == -1 (Error)
            {
                LOG("Failed to process HTTP response headers\n");
                return -1; // Propagate the error
            }
        }
        
        /* If headers are processed, we can now proceed to process the firmware data (response body) if available. 
         * This part is only reached if we have already processed the headers */
        if (http_headers_processed && (decrypted_data_pos < decrypted_data_len)) 
        {
            /* Count how many bytes we have left to process in the decrypted data buffer */
            size_t num_of_firmware_bytes_left = decrypted_data_len - decrypted_data_pos;
            
            /* Copy the received decrypted firmware data into the flash buffer. 
             * We keep track how much data we have written to the flash buffer - only after writing to flash the received bytes
             * count towards the total received bytes. */
            while (num_of_firmware_bytes_left > 0) 
            {
                size_t copy_size = CHUNK_SIZE - flash_buf_pos; // flash_buf_pos is 0'd out on each flush
                /* If we have less data than a full chunk, copy only the available data */
                if (copy_size > num_of_firmware_bytes_left) copy_size = num_of_firmware_bytes_left;
                
                /* Copy firmware data from decrypted data buffer to flash buffer */
                memcpy((p_flash_buffer + flash_buf_pos), (p_decrypted_data_buffer + decrypted_data_pos), copy_size);

                flash_buf_pos += copy_size; // move the position in the flash buffer by the number of bytes copied
                decrypted_data_pos += copy_size; // same for the decrypted data buffer

                num_of_firmware_bytes_left -= copy_size; // reduce the remaining bytes to process by the number of bytes copied
                
                /* We accumulate the decrypted firmware data in the flash buffer until it is full. If we have less than a full chunk,
                 * we exit the loop on after processing all available firmware bytes and return to the main loop to read more data from the socket. */
                if (flash_buf_pos == CHUNK_SIZE)
                {
                    /* Determine the base offset based on the inactive bank */
                    uint32_t base_offset = (inactive_bank == BANK_A) ? APP_BANK_A_OFFSET : APP_BANK_B_OFFSET;
                    uint32_t chunk_base_offset = base_offset + total_received;

                    LOG("Writing %d bytes to flash (Bank %c) at offset 0x%lx\n", CHUNK_SIZE, (inactive_bank == BANK_A ? 'A' : 'B'), chunk_base_offset);

                    /* Sector-by-sector flush. write_to_flash() disables interrupts for the full erase+program
                     * cycle. A 64 KB (16-sector) single call blocks interrupts for hundreds of milliseconds,
                     * long enough for the CYW43 chip's internal RX buffer to overflow and the lwIP tcpip_task
                     * to miss ACKs, which in turn kills the TLS stream mid-download. Flushing one sector at a
                     * time, with a short vTaskDelay between sectors, keeps the maximum interrupt-off window
                     * bounded (one sector erase+program ~20-50 ms) and gives the network stack a chance to
                     * drain the RX buffer between bursts. */
                    for (size_t sector_offset_in_buf = 0; sector_offset_in_buf < CHUNK_SIZE; sector_offset_in_buf += FLASH_SECTOR_SIZE)
                    {
                        uint32_t sector_write_offset = chunk_base_offset + sector_offset_in_buf;
                        if (!write_to_flash(sector_write_offset, p_flash_buffer + sector_offset_in_buf, FLASH_SECTOR_SIZE))
                        {
                            LOG("Flash write failed at sector offset 0x%lx\n", sector_write_offset);
                            return -1;
                        }
                        /* Yield so tcpip_task can ACK and CYW43 can drain its RX buffer. */
                        vTaskDelay(pdMS_TO_TICKS(2));
                    }

                    total_received += CHUNK_SIZE; // Only now update the total received bytes (in flash == received)
                    flash_buf_pos = 0; // Since we flushed the flash buffer, reset it for the next chunk
                }
                else
                {
                    LOG("Buffering %zu bytes in flash buffer - not a full CHUNK yet (which is %zu bytes)\n", flash_buf_pos, CHUNK_SIZE);
                }
            }
            /* Check for download completion based on expected_size *after* processing the chunk
             * Note: expected_size must be > 0 for this check to be meaningful */
            if ((expected_size > 0) && (total_received + flash_buf_pos >= expected_size)) 
            {
                /**
                 * Checks if the total amount of received OTA data, including data currently buffered but not yet written to flash, 
                 * meets or exceeds the expected size of the firmware update image.
                 * 
                 * To get an accurate picture of the total FIRMWARE data received so far, we must consider both the data
                 * that has already been successfully flushed to flash (`total_bytes_received`) and the data currently accumulated in 
                 * the intermediate buffer (`flash_buf_pos`) waiting for the next flush operation.
                 * 
                 * We do NOT count data in the TCP receive buffer, the decrypted data buffer, or the SSL buffer, as these are NOT
                 * necessarily firmware data. They may contain other data (e.g., HTTP headers, control messages, etc.) that we don't want to count. */
                LOG("Download complete condition met (received %zu + buffered %zu >= expected %zu)\n", 
                    total_received, flash_buf_pos, expected_size);

                /* The final write happens outside the loop in download_firmware (see write_last_chunk_to_flash function) */
                return 0; // Indicate download completion
            }
            else
            {
                LOG("Download not complete yet (received %zu + buffered %zu < expected %zu)\n", total_received, flash_buf_pos, expected_size);
                return 1; // Indicate more data is expected
            }
        }
        else
        {
            LOG("No firmware data to process after headers\n");
            return 1; // Indicate more data is expected
        }
    }
    /* Below we handle various SSL/TLS error codes. For reference look in ssh.h where these codes are defined */
    else if (p_ssl_read_status == 0) 
    {
        LOG("Connection closed by server (ssl_read returned 0)\n");
        /* Check if we received enough data *before* the connection closed */
        if ((expected_size > 0) && (total_received + flash_buf_pos >= expected_size)) 
        {
             LOG("Connection closed after receiving expected amount of data.\n");
             return 0; // Treat as complete, exit and let the main loop do the final write and finish the download
        } 
        else 
        {
             LOG("Connection closed prematurely. Expected %zu, got %zu + buffered %zu\n", expected_size, total_received, flash_buf_pos);
             return -2; // Indicate that we need to reconnect
        }
    } 
    else if (p_ssl_read_status == MBEDTLS_ERR_SSL_WANT_READ || p_ssl_read_status == MBEDTLS_ERR_SSL_WANT_WRITE) 
    {
        /* Non-blocking operation in progress, try again later. No log needed here, it's expected behavior */
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
        return 1; // Indicate that more data is expected/operation pending
    } 
    else if (p_ssl_read_status == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
    {
        /* Peer (OTA server) has sent a close notify alert, indicating graceful closure.
         * This happens when the server has finished sending data and is closing the connection.
         * It DOES NOT mean that we have successfully received and processed all the data.
         * We should still check if we have received all the expected data before exiting. */
        LOG("TLS connection gracefully closed by peer (close_notify received)\n");

        if ((expected_size > 0) && (total_received + flash_buf_pos >= expected_size))
        {
             LOG("Close notify received after receiving expected amount of data.\n");
             return 0; // Treat as complete, exit and let the main loop do the final write and finish the download
        } 
        else 
        {
             LOG("Close notify received prematurely. Expected %zu, got %zu + buffered %zu\n", expected_size, total_received, flash_buf_pos);
             return -2; // Indicate that we need to reconnect
        }
    }
    else if(p_ssl_read_status == MBEDTLS_ERR_SSL_TIMEOUT)
    {
        /* The underlying BIO read callback timed out waiting for data. Log the event, but let the outer loop's timeout_value handle
         * the overall timeout for the download process. */
        LOG("SSL read timeout occurred (MBEDTLS_ERR_SSL_TIMEOUT)\n");

        return 1; // Indicate that we are still expecting data (but might time out soon)
    }
    else if (p_ssl_read_status == MBEDTLS_ERR_NET_CONN_RESET) 
    {
        LOG("Connection reset by peer (MBEDTLS_ERR_NET_CONN_RESET)\n");
        return -2; // Signal main loop to attempt reconnect
    } 
    else 
    {
        /* Handle other SSL/TLS errors */
        LOG("SSL read error: %d (0x%04X)\n", p_ssl_read_status, -p_ssl_read_status);
        return -1; // General error
    }
}

static int attempt_reconnect(mbedtls_ssl_context *p_ssl_context, char *http_request)
{
    /* Note: reconnect_attempts is now file-scope (see top of file) so that download_firmware()
     * can reset it between invocations. */

    reconnect_attempts++;
    LOG("Connection reset, attempting to reconnect (%d/%d)...\n", reconnect_attempts, RECONNECT_MAX_ATTEMPTS);
    if (reconnect_attempts >= RECONNECT_MAX_ATTEMPTS) 
    {
        LOG("Maximum reconnection attempts reached\n");
        return -1; // Indicate failure
    }
    
    /* Reset an already initialized SSL context for re-use while retaining application-set variables, function pointers and data. */
    mbedtls_ssl_session_reset(p_ssl_context);
    
    /* Attempt to reconnect TCP client */
    if (tcp_client_is_connected()) 
    {
        tcp_client_disconnect();
    }
    
    /* Wait before reconnecting */
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if (!tcp_client_connect(OTA_HTTPS_SERVER_IP_ADDRESS, OTA_HTTPS_SERVER_PORT)) 
    {
        LOG("Failed to reconnect TCP client\n");
        return -1; // Indicate failure
    }
    
    /* Try handshake again */
    LOG("Performing handshake after reconnect...\n");

    timeout_value = xTaskGetTickCount() + pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS);

    /* Attempt to re-establish the TLS connection. 
     * This is similar to the initial handshake, but we don't need to set up the BIO again.
     * We just call mbedtls_ssl_handshake directly on the existing context. */

    int handshake_result = mbedtls_ssl_handshake(p_ssl_context);
    while (handshake_result != 0) 
    {
        /* Check for fatal errors, non-fatal "want" conditions or timeout */
        if ((handshake_result != MBEDTLS_ERR_SSL_WANT_READ) && (handshake_result != MBEDTLS_ERR_SSL_WANT_WRITE)) 
        {
            LOG("TLS handshake failed after reconnect: %08x\n", -handshake_result);
            return -1; // Indicate failure
        }
        else if (xTaskGetTickCount() >= timeout_value) 
        {
            LOG("TLS handshake timeout after reconnect\n");
            return -1; // Indicate failure
        }
        else
        {
            /* Continue the handshake process */
            handshake_result = mbedtls_ssl_handshake(p_ssl_context);
        }
        
        /* Give other tasks a chance to run while we wait for handshake data to be available */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    LOG("TLS handshake completed successfully after reconnect\n");

    /* Send HTTP http_request with Range header to resume download */
    size_t http_request_size = (size_t)HTTP_REQUEST_MAX_SIZE; 
    memset(http_request, 0, http_request_size); // Clean http_request variable first
    
    snprintf(http_request, http_request_size, 
        "GET %s HTTP/1.1\r\n"               // Specifies the GET method, the resource path, and the HTTP version.
        "Host: %s\r\n"                      // Specifies the host header with the server IP address.
        "Connection: close\r\n"             // Indicates that the TCP connection should be closed after the response (the remaining data).
        "Range: bytes=%zu-%zu\r\n"          // Specifies the range of bytes to request (resuming download).
        "Cache-Control: no-cache\r\n"       // Instructs the server not to use cached data (use fresh data).
        "User-Agent: Pico-W-OTA/1.0\r\n"    // Identifies the client making the request (optional).
        "\r\n",                             // End of headers (\r\n\r\n)
        FIRMWARE_PATH, OTA_HTTPS_SERVER_IP_ADDRESS, total_received, (expected_size - 1));
    
    /* Send the composed HTTP GET resume request to the server through the established secure TLS connection */
    LOG("Sending HTTP resume request: %s\n", http_request);
    if (mbedtls_ssl_write(p_ssl_context, (const unsigned char *)http_request, strlen(http_request)) <= 0) 
    {
        LOG("Failed to send HTTP resume request\n");
        return -1; /* Indicate failure */
    }
    
    /* Reset headers so that process_http_response_headers() can process the new response */
    http_headers_processed = false;
    
    /* Reset timeout */
    timeout_value = xTaskGetTickCount() + pdMS_TO_TICKS(READ_TIMEOUT_MS);

    return 0; // Indicate success
}

static int write_last_chunk_to_flash(uint8_t *p_flash_buffer, size_t p_flash_buf_pos)
{
    if (p_flash_buf_pos > 0)
    {
        /* Determine the base offset based on the inactive bank */
        uint32_t base_offset = (inactive_bank == BANK_A) ? APP_BANK_A_OFFSET : APP_BANK_B_OFFSET;

        /* Write the firmware data to the exact location where bootloader expects it (inactive bank) */
        uint32_t chunk_base_offset = base_offset + total_received;
        LOG("Writing final %zu bytes to flash (Bank %c) at offset 0x%lx\n", p_flash_buf_pos, (inactive_bank == BANK_A ? 'A' : 'B'), chunk_base_offset);

        /* Sector-by-sector flush, matching the main process_decrypted_data() loop. See the rationale
         * comment there: a single large write_to_flash() would disable interrupts too long and starve
         * the CYW43/lwIP path. The final chunk is < CHUNK_SIZE and typically not sector-aligned at the
         * tail, so the last iteration writes the remaining partial sector in one shot - write_to_flash()
         * already rounds the erase up to a full sector internally. */
        size_t bytes_flushed = 0;
        while (bytes_flushed < p_flash_buf_pos)
        {
            size_t bytes_left = p_flash_buf_pos - bytes_flushed;
            size_t this_write = (bytes_left >= FLASH_SECTOR_SIZE) ? FLASH_SECTOR_SIZE : bytes_left;
            uint32_t sector_write_offset = chunk_base_offset + bytes_flushed;
            if (!write_to_flash(sector_write_offset, p_flash_buffer + bytes_flushed, this_write))
            {
                LOG("Final flash write failed at sector offset 0x%lx\n", sector_write_offset);
                return -1; // Indicate failure
            }
            bytes_flushed += this_write;
            /* Yield so tcpip_task can run between sectors. Not strictly required for the final flush
             * (the stream is done), but keeps behavior consistent and avoids starving the network
             * stack during cleanup. */
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        total_received += p_flash_buf_pos; // update the total received bytes one final time (it should be equal to expected_size now)
    }

    return 0; // Indicate success
}

static void clean_up(mbedtls_ssl_context *p_ssl_context, mbedtls_ssl_config *p_ssl_config,
                     mbedtls_x509_crt *p_CA_cert_OTA_server, mbedtls_ctr_drbg_context *p_ctr_drbg_context,
                     mbedtls_entropy_context *p_entropy_context)
{
    /* Try to close the SSL connection gracefully if still connected */
    if (clientGlobal && tcp_client_is_connected())
    {
        LOG("Closing TLS connection...\n");
        /* Attempt graceful TLS closure. Ignore WANT_WRITE/READ in cleanup. */
        int ret = mbedtls_ssl_close_notify(p_ssl_context);
        if (ret != 0 && ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) 
        {
            /* Log the error but continue cleanup */
            LOG("mbedtls_ssl_close_notify failed: %d\n", ret);
        }

        /* Explicitly close the underlying TCP connection */
        LOG("Closing TCP connection...\n");
        tcp_client_disconnect();
    }

    /* Free all mbedTLS structures */
    mbedtls_ssl_free(p_ssl_context);
    mbedtls_ssl_config_free(p_ssl_config);
    mbedtls_x509_crt_free(p_CA_cert_OTA_server);
    mbedtls_ctr_drbg_free(p_ctr_drbg_context);
    mbedtls_entropy_free(p_entropy_context);

    watchdog_enable(((uint32_t)2000), true); // Re-enable watchdog after OTA download
}

#endif // OTA_ENABLED