# OTA Apache2 HTTPS Server Setup Guide

This guide covers setting up an Apache2 HTTPS server on Ubuntu/Debian for serving OTA firmware updates to the Pico2_W.

## Prerequisites

- Ubuntu or Debian-based Linux machine on the same network as the Pico
- `sudo` access
- `openssl` installed (usually pre-installed)

## 1. Install Apache2

```bash
sudo apt update
sudo apt install apache2
```

Verify it's running:

```bash
sudo systemctl status apache2
```

## 2. Enable Required Modules

Enable SSL, rate limiting, and headers modules:

```bash
sudo a2enmod ssl
sudo a2enmod ratelimit
sudo a2enmod headers
```

## 3. Generate a Self-Signed Certificate

The certificate's Common Name (CN) must match the IP address of the machine running the Apache2 server. This is the PC that will serve firmware files to the Pico over HTTPS.

To find your server's IP address on the local network:

```bash
hostname -I
```

Use the first address (typically something like `192.168.X.X`). Then run the helper script which generates the certificate and outputs the C string literal for the firmware code:

```bash
./Tools/GenerateOTACert.sh <server_ip>
```

For example, if `hostname -I` shows `192.168.50.194`:

```bash
./Tools/GenerateOTACert.sh 192.168.50.194
```

This will:
1. Generate a self-signed certificate at `/etc/ssl/certs/apache-selfsigned.crt`
2. Generate the private key at `/etc/ssl/private/apache-selfsigned.key`
3. Print the certificate as a C string literal to paste into `WiFi_OTA_Config.h`

**Optional: Manual certificate generation** (if you prefer not to use the script):

```bash
sudo openssl req -x509 -nodes \
    -days 365 \
    -newkey rsa:2048 \
    -keyout /etc/ssl/private/apache-selfsigned.key \
    -out /etc/ssl/certs/apache-selfsigned.crt \
    -subj "/C=PL/ST=Wielkopolska/L=Sieroszewice/O=Moduri/OU=OTA/CN=<your_server_ip>/emailAddress=vexown@gmail.com" \
    -addext "subjectAltName=IP:<your_server_ip>"
```

## 4. Configure Apache2 for HTTPS

Edit the default SSL site configuration:

```bash
sudo nano /etc/apache2/sites-available/default-ssl.conf
```

Ensure these lines point to your certificate and key:

```apache
SSLCertificateFile    /etc/ssl/certs/apache-selfsigned.crt
SSLCertificateKeyFile /etc/ssl/private/apache-selfsigned.key
```

## 5. Configure Rate Limiting

The Pico cannot process data as fast as the server can send it, so rate limiting is required.

In the same `default-ssl.conf` file, add the following block inside the `<VirtualHost>` section:

```apache
<Location /firmware.bin>
    SetOutputFilter RATE_LIMIT
    # Set the download speed in KB/s
    SetEnv rate-limit 75

    # Prevent caching to ensure fresh firmware is always served
    Header set Cache-Control "no-cache, no-store, must-revalidate"
    Header set Pragma "no-cache"
    Header set Expires 0
</Location>
```

## 6. Enable the SSL Site and Restart

```bash
sudo a2ensite default-ssl
sudo apache2ctl configtest
sudo systemctl restart apache2
```

## 7. Upload Firmware

Use the provided upload script:

```bash
./UploadImageToServer.sh
```

This copies the compiled binary from `./output/moduri_bank_B.bin` to `/var/www/html/firmware.bin` and restarts Apache.

Alternatively, to upload manually:

```bash
sudo cp ./output/moduri_bank_B.bin /var/www/html/firmware.bin
sudo chown www-data:www-data /var/www/html/firmware.bin
sudo chmod 644 /var/www/html/firmware.bin
sudo systemctl restart apache2
```

## 8. Update Firmware Code Configuration

After generating a new certificate or changing networks, update the firmware source code:

1. **Copy the C-formatted certificate** from the `GenerateOTACert.sh` output into `Application/WiFi/Include/WiFi_OTA_Config.h` (replace `CA_cert_OTA_server_raw`)

2. **Update the server IP** in the same file:
   ```c
   #define OTA_HTTPS_SERVER_IP_ADDRESS  "192.168.50.194"
   ```

3. **Update network addresses** in `Application/WiFi/Include/WiFi_Common.h` if the subnet changed:
   ```c
   #define REMOTE_TCP_SERVER_IP_ADDRESS "192.168.50.194"
   #define PICO_W_STATIC_IP_ADDRESS     "192.168.50.50"
   #define GATEWAY_ADDR                 "192.168.50.1"
   ```

4. **Update WiFi credentials** in `Application/WiFi/Include/WiFi_Credentials.h` if the network changed.

5. **Rebuild** the firmware.

## 9. Renewing or Replacing the Certificate

You need to regenerate the certificate when:
- The certificate has expired (default validity is 365 days)
- The server IP address changed (e.g. new router, different machine)
- You moved the OTA server to a different machine

Steps:

1. **Regenerate the certificate** on the server machine:
   ```bash
   ./Tools/GenerateOTACert.sh <new_server_ip>
   ```
   The script will overwrite the old cert/key, restart Apache, and print the new C string.

2. **Update `WiFi_OTA_Config.h`**:
   - Replace `CA_cert_OTA_server_raw` with the C string output from the script
   - Update `OTA_HTTPS_SERVER_IP_ADDRESS` if the IP changed

3. **Update `WiFi_Common.h`** if the subnet changed (static IP, gateway, netmask, remote TCP server IP).

4. **Rebuild and flash** the firmware to the Pico.

To check when the current server certificate expires:
```bash
openssl x509 -in /etc/ssl/certs/apache-selfsigned.crt -noout -dates
```

## 10. Testing

Verify the HTTPS server is accessible:

```bash
curl -k https://<your_server_ip>/firmware.bin -o downloaded_firmware.bin
```

Verify the rate limit is working:

```bash
curl -o /dev/null -s -w "Time: %{time_total}s\nSpeed: %{speed_download} bytes/sec\n" \
    https://<your_server_ip>/firmware.bin -k
```

At 75 KB/s rate limit, a 500 KB firmware should take roughly 7 seconds.

## 10. Managing the Server

```bash
# Start the server
sudo systemctl start apache2

# Stop the server
sudo systemctl stop apache2

# Check status
sudo systemctl status apache2

# Restart (after config changes or firmware upload)
sudo systemctl restart apache2
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| TLS handshake fails on Pico | Certificate CN/SAN must match `OTA_HTTPS_SERVER_IP_ADDRESS` in `WiFi_OTA_Config.h`. Regenerate the cert with the correct IP. |
| Certificate expired | Default validity is 365 days. Regenerate with `./Tools/GenerateOTACert.sh <ip>` and update `WiFi_OTA_Config.h`. |
| Connection refused | Check Apache is running (`systemctl status apache2`) and the SSL site is enabled (`sudo a2ensite default-ssl`). |
| Download too slow or times out | Check the rate limit setting in `default-ssl.conf`. 75 KB/s is the tested safe value. |
| "Permission denied" serving firmware | Run `sudo chown www-data:www-data /var/www/html/firmware.bin && sudo chmod 644 /var/www/html/firmware.bin`. |
| Pico can't reach server | Verify both devices are on the same subnet. Check `PICO_W_STATIC_IP_ADDRESS`, `NETMASK_ADDR`, and `GATEWAY_ADDR` in `WiFi_Common.h`. |
