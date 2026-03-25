/**
 * File: main.c - H-Bridge Motor Controller Example
 * Description: Demonstrates how to use the H-bridge controller driver to
 *              drive a DC motor forward, reverse, and at varying speeds.
 */

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "h_bridge_controller.h"

static void print_status(const char *label, HBridge_Status status)
{
    if (status == HBRIDGE_OK) printf("  [OK]  %s\n", label);
    else printf("  [ERR] %s (status=%d)\n", label, status);
}

int main(void)
{
    stdio_init_all();

    printf("\nH-Bridge Motor Controller Example\n");
    printf("----------------------------------\n\n");

    /* Initialize the H-bridge driver */
    printf("Initializing H-Bridge...\n");
    HBridge_Status status = HBridge_Init();
    if (status != HBRIDGE_OK)
    {
        printf("ERROR: H-Bridge initialization failed (status=%d).\n", status);
        return 1;
    }
    printf("H-Bridge initialized successfully.\n\n");

    /* Enable both sides of the H-bridge */
    status = HBridge_SetEnable(HBRIDGE_SIDE_BOTH, true);
    print_status("Enable both sides", status);

    /* --- Forward at 50% speed for 2 seconds --- */
    printf("\n--- Forward at 50%% speed ---\n");
    status = HBridge_RunFor(128, HBRIDGE_FORWARD, 2000);
    print_status("RunFor (forward, 50%%, 2s)", status);

    sleep_ms(500);

    /* --- Reverse at 75% speed for 2 seconds --- */
    printf("\n--- Reverse at 75%% speed ---\n");
    status = HBridge_RunFor(191, HBRIDGE_REVERSE, 2000);
    print_status("RunFor (reverse, 75%%, 2s)", status);

    sleep_ms(500);

    /* --- Ramp up speed forward --- */
    printf("\n--- Speed ramp (forward) ---\n");
    for (uint8_t speed = 0; speed < 255; speed += 25)
    {
        printf("  Speed: %3d/255\n", speed);
        HBridge_SetSpeed(speed, HBRIDGE_FORWARD);
        sleep_ms(500);
    }

    /* Full speed briefly */
    printf("  Speed: 255/255\n");
    HBridge_SetSpeed(255, HBRIDGE_FORWARD);
    sleep_ms(1000);

    /* Stop the motor */
    printf("\n--- Stopping motor ---\n");
    status = HBridge_Stop();
    print_status("Stop", status);

    /* Disable the H-bridge */
    status = HBridge_SetEnable(HBRIDGE_SIDE_BOTH, false);
    print_status("Disable both sides", status);

    printf("\nDone.\n");
    return 0;
}
