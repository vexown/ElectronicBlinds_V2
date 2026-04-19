/*
 * @file button_debouncer.c
 * @brief Shift-register debouncer implementation.
 */

#include "button_debouncer.h"
#include "pico/stdlib.h"

/* -----------------------------------------------------------------------
 * Internal state for each registered button
 */
typedef struct
{
    uint8_t  pin;
    bool     active_low;
    uint8_t  history;      /* 8-bit shift register of raw samples         */
    bool     held;         /* current debounced state (true = pressed)     */
    volatile bool pressed_flag;   /* set by ISR, cleared by Debounce_Pressed()  */
    volatile bool released_flag;  /* set by ISR, cleared by Debounce_Released() */
    bool     in_use;
} Debounce_Entry;

static Debounce_Entry s_buttons[DEBOUNCE_MAX_BUTTONS];
static uint8_t        s_count = 0;
static bool           s_initialized = false;
static struct repeating_timer s_timer;

/* -----------------------------------------------------------------------
 * Helpers
 */

static Debounce_Entry *find_entry(uint8_t pin)
{
    for (uint8_t i = 0; i < s_count; i++)
    {
        if (s_buttons[i].in_use && s_buttons[i].pin == pin) return &s_buttons[i];
    }
    return NULL;
}

/* -----------------------------------------------------------------------
 * Timer callback — runs every DEBOUNCE_SAMPLE_MS in interrupt context
 */
static bool debounce_timer_cb(struct repeating_timer *t)
{
    (void)t;

    for (uint8_t i = 0; i < s_count; i++)
    {
        Debounce_Entry *btn = &s_buttons[i];
        if (!btn->in_use) continue;

        /* Read the raw pin level */
        GPIO_State raw;
        GPIO_Read(btn->pin, &raw);

        /* Shift history left, roll in new sample as bit 0 */
        btn->history = (btn->history << 1) | (raw == GPIO_HIGH ? 1 : 0);

        /*
         * Apply the mask and check for transitions.
         *
         * For active-low buttons (pull-up, reads LOW when pressed):
         *   Press  : history was 0b11xxx000  →  masked = 0b11000000 = 0xC0
         *   Release: history was 0b00xxx111  →  masked = 0b00000111 = 0x07
         *
         * For active-high buttons (pull-down, reads HIGH when pressed):
         *   Press  : history was 0b00xxx111  →  masked = 0x07
         *   Release: history was 0b11xxx000  →  masked = 0xC0
         */
        uint8_t masked = btn->history & DEBOUNCE_MASK;

        uint8_t press_pattern;
        uint8_t release_pattern;

        if (btn->active_low)
        {
            press_pattern   = 0xC0; /* 0b11000000 — was high, now low  */
            release_pattern = 0x07; /* 0b00000111 — was low, now high  */
        }
        else
        {
            press_pattern   = 0x07; /* 0b00000111 — was low, now high  */
            release_pattern = 0xC0; /* 0b11000000 — was high, now low  */
        }

        if (masked == press_pattern && !btn->held)
        {
            btn->held = true;
            btn->pressed_flag = true;
            /* Reset history to solid "pressed" state to create hysteresis */
            btn->history = btn->active_low ? 0x00 : 0xFF;
        }
        else if (masked == release_pattern && btn->held)
        {
            btn->held = false;
            btn->released_flag = true;
            /* Reset history to solid "released" state */
            btn->history = btn->active_low ? 0xFF : 0x00;
        }
    }

    return true; /* keep the timer running */
}

/* -----------------------------------------------------------------------
 * Public API
 */

Debounce_Status Debounce_Init(void)
{
    if (s_initialized) return DEBOUNCE_OK;

    for (uint8_t i = 0; i < DEBOUNCE_MAX_BUTTONS; i++)
    {
        s_buttons[i].in_use = false;
    }
    s_count = 0;

    /* Start the repeating timer (negative value = delay between callbacks) */
    if (!add_repeating_timer_ms(-DEBOUNCE_SAMPLE_MS, debounce_timer_cb, NULL, &s_timer))
    {
        return DEBOUNCE_ERROR_NOT_INITIALIZED;
    }

    s_initialized = true;
    return DEBOUNCE_OK;
}

Debounce_Status Debounce_AddButton(uint8_t pin, GPIO_Pull pull, bool active_low)
{
    if (!s_initialized) return DEBOUNCE_ERROR_NOT_INITIALIZED;
    if (s_count >= DEBOUNCE_MAX_BUTTONS) return DEBOUNCE_ERROR_MAX_BUTTONS;

    /* Don't register the same pin twice */
    if (find_entry(pin)) return DEBOUNCE_OK;

    /* Configure the GPIO as input with the requested pull */
    if (GPIO_Init(pin, GPIO_INPUT, pull) != GPIO_OK) return DEBOUNCE_ERROR_INVALID_PIN;

    Debounce_Entry *btn = &s_buttons[s_count];
    btn->pin           = pin;
    btn->active_low    = active_low;
    btn->history       = active_low ? 0xFF : 0x00; /* assume idle state */
    btn->held          = false;
    btn->pressed_flag  = false;
    btn->released_flag = false;
    btn->in_use        = true;
    s_count++;

    return DEBOUNCE_OK;
}

bool Debounce_Pressed(uint8_t pin)
{
    Debounce_Entry *btn = find_entry(pin);
    if (!btn) return false;
    if (btn->pressed_flag)
    {
        btn->pressed_flag = false;
        return true;
    }
    return false;
}

bool Debounce_Released(uint8_t pin)
{
    Debounce_Entry *btn = find_entry(pin);
    if (!btn) return false;
    if (btn->released_flag)
    {
        btn->released_flag = false;
        return true;
    }
    return false;
}

bool Debounce_IsHeld(uint8_t pin)
{
    Debounce_Entry *btn = find_entry(pin);
    if (!btn) return false;
    return btn->held;
}
