#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define SERVO1_GPIO 18 // Servo 1 control pin
#define SERVO2_GPIO 19 // Servo 2 control pin

#define SERVO_PERIOD_US 20000 // 20 ms period (50 Hz)
#define SERVO_MIN_PULSE_US 1250
#define SERVO_MAX_PULSE_US 2000
static const char *TAG = "dual_servo";

// ---------------------- Servo 1 Globals ----------------------
static volatile uint32_t current_pulse_us1 = 1500; // Start at mid (90°)
static volatile int pwm_state1 = 0;                // 0: low-phase, 1: high-phase
static esp_timer_handle_t pwm_timer1;

// ---------------------- Servo 2 Globals ----------------------
// For servo2, we want to start at 0°.
static volatile uint32_t current_pulse_us2 = SERVO_MIN_PULSE_US;
static volatile int pwm_state2 = 0;
static esp_timer_handle_t pwm_timer2;

// ---------------------- Servo 1 Timer Callback ----------------------
static void IRAM_ATTR pwm_timer_callback1(void *arg)
{
    if (pwm_state1 == 0)
    {
        gpio_set_level(SERVO1_GPIO, 1);
        esp_timer_start_once(pwm_timer1, current_pulse_us1);
        pwm_state1 = 1;
    }
    else
    {
        gpio_set_level(SERVO1_GPIO, 0);
        esp_timer_start_once(pwm_timer1, SERVO_PERIOD_US - current_pulse_us1);
        pwm_state1 = 0;
    }
}

// ---------------------- Servo 2 Timer Callback ----------------------
static void IRAM_ATTR pwm_timer_callback2(void *arg)
{
    if (pwm_state2 == 0)
    {
        gpio_set_level(SERVO2_GPIO, 1);
        esp_timer_start_once(pwm_timer2, current_pulse_us2);
        pwm_state2 = 1;
    }
    else
    {
        gpio_set_level(SERVO2_GPIO, 0);
        esp_timer_start_once(pwm_timer2, SERVO_PERIOD_US - current_pulse_us2);
        pwm_state2 = 0;
    }
}

// ---------------------- Smooth Transition for Servo 1 ----------------------
void smooth_transition_servo1(uint32_t start_pulse, uint32_t end_pulse, uint32_t transition_time_ms)
{
    int steps = 20; // Number of increments for smooth motion
    int32_t delta = (int32_t)end_pulse - (int32_t)start_pulse;
    int32_t step_size = delta / steps;
    uint32_t delay_per_step = transition_time_ms / steps;

    for (int i = 0; i < steps; i++)
    {
        current_pulse_us1 = start_pulse + (step_size * i);
        vTaskDelay(pdMS_TO_TICKS(delay_per_step));
    }
    current_pulse_us1 = end_pulse;
}

// ---------------------- Smooth Transition for Servo 2 ----------------------
void smooth_transition_servo2(uint32_t start_pulse, uint32_t end_pulse, uint32_t transition_time_ms)
{
    int steps = 20; // Use a fixed number of increments
    int32_t delta = (int32_t)end_pulse - (int32_t)start_pulse;
    int32_t step_size = delta / steps;
    uint32_t delay_per_step = transition_time_ms / steps;

    for (int i = 0; i < steps; i++)
    {
        current_pulse_us2 = start_pulse + (step_size * i);
        vTaskDelay(pdMS_TO_TICKS(delay_per_step));
    }
    current_pulse_us2 = end_pulse;
}

void app_main(void)
{
    // ---------------------- Initialize GPIOs ----------------------
    gpio_reset_pin(SERVO1_GPIO);
    gpio_set_direction(SERVO1_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SERVO2_GPIO);
    gpio_set_direction(SERVO2_GPIO, GPIO_MODE_OUTPUT);

    // ---------------------- Create Timers for Both Servos ----------------------
    const esp_timer_create_args_t timer_args1 = {
        .callback = pwm_timer_callback1,
        .name = "pwm_timer1"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args1, &pwm_timer1));
    esp_timer_start_once(pwm_timer1, 0);

    const esp_timer_create_args_t timer_args2 = {
        .callback = pwm_timer_callback2,
        .name = "pwm_timer2"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args2, &pwm_timer2));
    esp_timer_start_once(pwm_timer2, 0);

    // ---------------------- Transition Parameters for Servo 1 ----------------------
    uint32_t fast_time_ms = 100;                          // Fast transition: 0° -> 180° in 100 ms
    uint32_t slow_time_ms = 100;                          // Slow transition: 180° -> 0° in 2000 ms
    float fast_speed = 180.0f / (fast_time_ms / 1000.0f); // deg/s for fast motion
    float slow_speed = 180.0f / (slow_time_ms / 1000.0f); // deg/s for slow motion

    // ---------------------- Transition Parameter for Servo 2 ----------------------
    // For servo2, a full transition from 0° to 180° over 10 seconds (10000 ms)
    uint32_t servo2_transition_time_ms = 1000;
    float servo2_speed = 180.0f / (servo2_transition_time_ms / 1000.0f);

    while (1)
    {

        // ------------------ Servo 2: Transition from 0° to 180° over 10 seconds ------------------
        ESP_LOGI(TAG, "Servo2: 0° -> 180° in %" PRIu32 " ms (approx. %.2f deg/s)", servo2_transition_time_ms, servo2_speed);
        smooth_transition_servo2(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US, servo2_transition_time_ms);
        vTaskDelay(pdMS_TO_TICKS(4000));

        // ------------------ Servo 1: Fast transition from 0° to 180° ------------------
        ESP_LOGI(TAG, "Servo1 fast: 180° in %" PRIu32 " ms (approx. %.2f deg/s)", fast_time_ms, fast_speed);
        smooth_transition_servo1(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US, fast_time_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // ------------------ Servo 1: Slow transition from 180° back to 0° ------------------
        ESP_LOGI(TAG, "Servo1 slow: 180° in %" PRIu32 " ms (approx. %.2f deg/s)", slow_time_ms, slow_speed);
        smooth_transition_servo1(SERVO_MAX_PULSE_US, SERVO_MIN_PULSE_US, slow_time_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Optionally, reverse Servo 2 back to 0° over 10 seconds to repeat the cycle:
        ESP_LOGI(TAG, "Servo2: 180° -> 0° in %" PRIu32 " ms (approx. %.2f deg/s)", servo2_transition_time_ms, servo2_speed);
        smooth_transition_servo2(SERVO_MAX_PULSE_US, SERVO_MIN_PULSE_US, servo2_transition_time_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
