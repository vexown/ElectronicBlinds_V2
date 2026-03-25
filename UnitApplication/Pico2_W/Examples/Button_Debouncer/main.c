/*
 * File: main.c - Button Debouncer Example
 * Description: Demonstrates the shift-register debouncer with an active-low
 *              button on GP15 (pull-up) and an LED on GP25.
 */

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "button_debouncer.h"
#include "GPIO_HAL.h"

#define BUTTON_PIN  15
#define LED_PIN     25

int main(void)
{
    stdio_init_all();

    printf("\nButton Debouncer Example\n");
    printf("------------------------\n\n");

    /* Set up the LED */
    GPIO_Init(LED_PIN, GPIO_OUTPUT, GPIO_PULL_NONE);
    GPIO_Clear(LED_PIN);

    /* Initialize the debouncer and register our button */
    Debounce_Status status = Debounce_Init();
    if (status != DEBOUNCE_OK)
    {
        printf("ERROR: Debounce_Init failed (status=%d).\n", status);
        return 1;
    }

    status = Debounce_AddButton(BUTTON_PIN, GPIO_PULL_UP, true);
    if (status != DEBOUNCE_OK)
    {
        printf("ERROR: Debounce_AddButton failed (status=%d).\n", status);
        return 1;
    }
    printf("Button on GP%d registered (active-low, pull-up).\n", BUTTON_PIN);
    printf("Sampling every %d ms, mask 0x%02X.\n\n", DEBOUNCE_SAMPLE_MS, DEBOUNCE_MASK);

    uint32_t press_count = 0;

    while (1)
    {
        if (Debounce_Pressed(BUTTON_PIN))
        {
            press_count++;
            GPIO_Set(LED_PIN);
            printf("[%lu] Button PRESSED\n", press_count);
        }

        if (Debounce_Released(BUTTON_PIN))
        {
            GPIO_Clear(LED_PIN);
            printf("[%lu] Button RELEASED\n", press_count);
        }

        /* Nothing else to do — debouncing runs in the background via timer ISR */
        tight_loop_contents();
    }
}
