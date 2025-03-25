#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define SERVO_GPIO 18           // Your servo control pin
#define SERVO_PERIOD_US 20000   // 20 ms period (50 Hz)
#define SERVO_MIN_PULSE_US 1000 // 1 ms pulse width for 0° position
#define SERVO_MAX_PULSE_US 2000 // 2 ms pulse width for 180° position

static const char *TAG = "software_pwm";

// Global variable to hold the current pulse width (in microseconds)
// Adjust this value to change the servo angle.
static volatile uint32_t current_pulse_us = 1500; // Mid position

// State: 0 = waiting to start the high pulse, 1 = waiting to end it.
static volatile int pwm_state = 0;

// Timer handle for our PWM timer.
static esp_timer_handle_t pwm_timer;

// Timer callback function: toggles the GPIO and schedules the next transition.
static void IRAM_ATTR pwm_timer_callback(void *arg)
{
    if (pwm_state == 0)
    {
        // Start of the PWM cycle: set the servo pin HIGH.
        gpio_set_level(SERVO_GPIO, 1);
        // Schedule callback after the high pulse duration.
        esp_timer_start_once(pwm_timer, current_pulse_us);
        pwm_state = 1;
    }
    else
    {
        // End of the high pulse: set the servo pin LOW.
        gpio_set_level(SERVO_GPIO, 0);
        // Schedule callback after the remaining period.
        esp_timer_start_once(pwm_timer, SERVO_PERIOD_US - current_pulse_us);
        pwm_state = 0;
    }
}

void app_main(void)
{
    // Configure the GPIO pin for output.
    gpio_reset_pin(SERVO_GPIO);
    gpio_set_direction(SERVO_GPIO, GPIO_MODE_OUTPUT);

    // Create the high-resolution timer.
    const esp_timer_create_args_t timer_args = {
        .callback = pwm_timer_callback,
        .name = "pwm_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pwm_timer));

    // Start the PWM cycle immediately.
    esp_timer_start_once(pwm_timer, 0);

    // For demonstration: alternate the servo position every 2 seconds.
    // You can adjust the current_pulse_us value to control the angle.
    while (1)
    {
        ESP_LOGI(TAG, "Setting servo to MIN position (0°)");
        current_pulse_us = SERVO_MIN_PULSE_US;
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Setting servo to MAX position (180°)");
        current_pulse_us = SERVO_MAX_PULSE_US;
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Setting servo to MID position (90°)");
        current_pulse_us = (SERVO_MIN_PULSE_US + SERVO_MAX_PULSE_US) / 2;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
