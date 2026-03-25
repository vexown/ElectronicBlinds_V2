#include "PWM_HAL.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
#define MAX_GPIO_PIN 29
#define MIN_FREQUENCY 1
#define MAX_FREQUENCY 62500000
#define MIN_DUTY 0.0f
#define MAX_DUTY 100.0f

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
static bool pwm_initialized[MAX_GPIO_PIN + 1] = {false};
static uint32_t current_frequency[MAX_GPIO_PIN + 1] = {0};
static uint16_t slice_wrap_values[NUM_PWM_SLICES] = {0};

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/
static bool is_valid_pin(uint8_t pin);
static bool is_valid_frequency(uint32_t freq);
static bool is_valid_duty_cycle(float duty);

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/
static bool is_valid_pin(uint8_t pin) 
{
    return pin <= MAX_GPIO_PIN;
}

static bool is_valid_frequency(uint32_t freq) 
{
    return (freq >= MIN_FREQUENCY && freq <= MAX_FREQUENCY);
}

static bool is_valid_duty_cycle(float duty) 
{
    return (duty >= MIN_DUTY && duty <= MAX_DUTY);
}

static bool is_valid_slice(uint slice_num) 
{
    return slice_num < NUM_PWM_SLICES;
}

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/
PWM_Status PWM_Init(const PWM_Config* config) 
{
    if (config == NULL) return PWM_ERROR_NULL_POINTER;
    if (!is_valid_slice(pwm_gpio_to_slice_num(config->pin))) return PWM_ERROR_INVALID_PIN;
    if (!is_valid_pin(config->pin)) return PWM_ERROR_INVALID_PIN;
    if (!is_valid_frequency(config->frequency)) return PWM_ERROR_INVALID_FREQUENCY;
    if (!is_valid_duty_cycle(config->duty_cycle)) return PWM_ERROR_INVALID_DUTY;

    gpio_set_function(config->pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(config->pin);
    uint channel = pwm_gpio_to_channel(config->pin);

    uint32_t system_clock = clock_get_hz(clk_sys);
    float divider = (float)system_clock / ((float)config->frequency * 65535.0f);
    uint16_t wrap = (uint16_t)((float)system_clock / ((float)config->frequency * divider) - 1.0f);

    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, divider);
    pwm_config_set_wrap(&pwm_cfg, wrap);
    
    if (config->invert_output) {
        pwm_config_set_output_polarity(&pwm_cfg, channel == PWM_CHAN_A, channel == PWM_CHAN_B);
    }

    pwm_init(slice_num, &pwm_cfg, false);

    // Store wrap value and update state
    slice_wrap_values[slice_num] = wrap;
    uint16_t level = (uint16_t)((config->duty_cycle / 100.0f) * (wrap + 1));
    pwm_set_chan_level(slice_num, channel, level);

    pwm_initialized[config->pin] = true;
    current_frequency[config->pin] = config->frequency;

    return PWM_OK;
}

PWM_Status PWM_Start(uint8_t pin) 
{
    if (!is_valid_pin(pin)) return PWM_ERROR_INVALID_PIN;
    if (!pwm_initialized[pin]) return PWM_ERROR_NOT_INITIALIZED;

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, true);
    return PWM_OK;
}

PWM_Status PWM_Stop(uint8_t pin) 
{
    if (!is_valid_pin(pin)) return PWM_ERROR_INVALID_PIN;
    if (!pwm_initialized[pin]) return PWM_ERROR_NOT_INITIALIZED;

    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, false);
    return PWM_OK;
}

PWM_Status PWM_SetDutyCycle(uint8_t pin, float duty_cycle) 
{
    if (!is_valid_pin(pin)) return PWM_ERROR_INVALID_PIN;
    if (!pwm_initialized[pin]) return PWM_ERROR_NOT_INITIALIZED;
    if (!is_valid_duty_cycle(duty_cycle)) return PWM_ERROR_INVALID_DUTY;

    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    uint16_t wrap = slice_wrap_values[slice_num];
    uint16_t level = (uint16_t)((duty_cycle / 100.0f) * (wrap + 1));
    pwm_set_chan_level(slice_num, channel, level);
    return PWM_OK;
}

PWM_Status PWM_SetFrequency(uint8_t pin, uint32_t frequency) 
{
    if (!is_valid_pin(pin)) return PWM_ERROR_INVALID_PIN;
    if (!pwm_initialized[pin]) return PWM_ERROR_NOT_INITIALIZED;
    if (!is_valid_frequency(frequency)) return PWM_ERROR_INVALID_FREQUENCY;

    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t system_clock = clock_get_hz(clk_sys);
    float divider = (float)system_clock / ((float)frequency * 65535.0f);
    uint16_t wrap = (uint16_t)((float)system_clock / ((float)frequency * divider) - 1.0f);

    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, divider);
    pwm_config_set_wrap(&pwm_cfg, wrap);
    
    // Update stored wrap value
    slice_wrap_values[slice_num] = wrap;
    
    pwm_init(slice_num, &pwm_cfg, true);
    current_frequency[pin] = frequency;
    return PWM_OK;
}

PWM_Status PWM_DeInit(uint8_t pin) 
{
    if (!is_valid_pin(pin)) return PWM_ERROR_INVALID_PIN;
    if (!pwm_initialized[pin]) return PWM_ERROR_NOT_INITIALIZED;

    PWM_Stop(pin);
    gpio_set_function(pin, GPIO_FUNC_NULL);
    pwm_initialized[pin] = false;
    current_frequency[pin] = 0;
    slice_wrap_values[pwm_gpio_to_slice_num(pin)] = 0;
    return PWM_OK;
}