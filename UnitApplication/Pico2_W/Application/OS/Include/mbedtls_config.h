#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* Include FreeRTOS for memory management */
#include "FreeRTOS.h"
#include <stdio.h>

/* TODO - Document this config file based on mbedtls/config.h from pico-sdk */

/* System support */
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_HAVE_ASM
#define MBEDTLS_PLATFORM_NO_STD_FUNCTIONS

/* Define custom snprintf for debug */
#define MBEDTLS_PLATFORM_SNPRINTF_MACRO snprintf  // Use Newlib’s snprintf if available

/* Base crypto modules */
#define MBEDTLS_AES_C
#define MBEDTLS_SHA1_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_MD_C
#define MBEDTLS_MD5_C
#define MBEDTLS_BIGNUM_C

/* ASN.1 and object parsing */
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_OID_C
#define MBEDTLS_BASE64_C

/* Add this line to enable PEM parsing */
#define MBEDTLS_PEM_PARSE_C

/* PKCS support */
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_PKCS5_C

/* Public key cryptography */
#define MBEDTLS_RSA_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED

/* Key exchange methods */
#define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED

/* X.509 certificates */
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C

/* TLS/SSL */
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_MAX_CONTENT_LEN 16384 

/* Cipher modes */
#define MBEDTLS_CCM_C
#define MBEDTLS_GCM_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_CIPHER_MODE_CTR
#define MBEDTLS_CIPHER_PADDING_PKCS7

/* Random number generation */
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT

/* Memory allocation */
#define MBEDTLS_PLATFORM_CALLOC_MACRO pvPortCalloc
#define MBEDTLS_PLATFORM_FREE_MACRO   vPortFree

/* System configuration - disabled for embedded */
#define MBEDTLS_HAVE_TIME

/* Enable debugging */
#define MBEDTLS_DEBUG_C

#include "mbedtls/check_config.h"

#endif /* MBEDTLS_CONFIG_H */