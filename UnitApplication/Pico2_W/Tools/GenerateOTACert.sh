#!/bin/bash

# ****************************************************************************
# GenerateOTACert.sh
#
# Description:
# Generates a self-signed certificate for the Apache2 OTA server and outputs
# the certificate as a C string literal ready to paste into WiFi_OTA_Config.h.
#
# Usage:
#   ./Tools/GenerateOTACert.sh <server_ip> [validity_days]
#
# Arguments:
#   server_ip      - IP address of the Apache2 OTA server (e.g. 192.168.50.194)
#   validity_days  - (Optional) Certificate validity in days, default: 365
#
# Output files (requires sudo):
#   /etc/ssl/certs/apache-selfsigned.crt   - Certificate
#   /etc/ssl/private/apache-selfsigned.key - Private key
#
# For full Apache2 setup instructions, see Docs/OTA_Apache2_Setup.md
# ****************************************************************************

set -euo pipefail

# ---- Argument parsing ----
if [ $# -lt 1 ]; then
    echo "Usage: $0 <server_ip> [validity_days]"
    echo "Example: $0 192.168.50.194"
    echo "Example: $0 192.168.50.194 730"
    exit 1
fi

SERVER_IP="$1"
VALIDITY_DAYS="${2:-365}"
CERT_PATH="/etc/ssl/certs/apache-selfsigned.crt"
KEY_PATH="/etc/ssl/private/apache-selfsigned.key"

# ---- Validate IP format (basic check) ----
if ! echo "$SERVER_IP" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$'; then
    echo "Error: '$SERVER_IP' does not look like a valid IPv4 address."
    exit 1
fi

# ---- Generate the self-signed certificate ----
echo "Generating self-signed certificate for $SERVER_IP (valid for $VALIDITY_DAYS days)..."

sudo openssl req -x509 -nodes \
    -days "$VALIDITY_DAYS" \
    -newkey rsa:2048 \
    -keyout "$KEY_PATH" \
    -out "$CERT_PATH" \
    -subj "/O=Moduri/OU=OTA/CN=$SERVER_IP" \
    -addext "subjectAltName=IP:$SERVER_IP"

echo ""
echo "Certificate written to: $CERT_PATH"
echo "Private key written to: $KEY_PATH"
echo ""

# ---- Restart Apache2 if running ----
if systemctl is-active --quiet apache2; then
    echo "Apache2 is running. Restarting to pick up the new certificate..."
    sudo systemctl restart apache2
    echo "Apache2 restarted."
    echo ""
fi

# ---- Output the certificate as a C string literal ----
echo "============================================================"
echo "Copy the following into WiFi_OTA_Config.h (CA_cert_OTA_server_raw):"
echo "============================================================"
echo ""
echo 'static const unsigned char *CA_cert_OTA_server_raw ='
echo '    (const unsigned char *)'

# Read the cert and format each line as a C string literal
while IFS= read -r line; do
    echo "    \"${line}\n\""
done < "$CERT_PATH"

echo '    ;'
echo ""
echo "============================================================"
echo "Also update OTA_HTTPS_SERVER_IP_ADDRESS in WiFi_OTA_Config.h:"
echo "  #define OTA_HTTPS_SERVER_IP_ADDRESS  \"$SERVER_IP\""
echo "============================================================"
