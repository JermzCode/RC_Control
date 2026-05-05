//Controls
//  w = slow forward
//  s = slow reverse
//  x = stop
//  a = full left
//  d = full right
//  c = steering center
//  i = throttle +5 us
//  k = throttle -5 us
//  j = steer left -30 us
//  l = steer right +30 us
//  idf.py -p COM5 flash monitor
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"

// GPIO pins
#define ESC_GPIO                17
#define SERVO_GPIO              18

// PWM timing
#define TIMER_RESOLUTION_HZ     1000000   // 1 tick = 1 us
#define PERIOD_US               20000     // 20 ms = 50 Hz

// Throttle calibration
#define THROTTLE_REVERSE_LIMIT  1350
#define THROTTLE_REVERSE_SLOW   1415
#define THROTTLE_NEUTRAL        1500
#define THROTTLE_FORWARD_SLOW   1590
#define THROTTLE_FORWARD_LIMIT  1650
#define THROTTLE_FINE_STEP      5

// Steering calibration
#define STEERING_LEFT_LIMIT     800
#define STEERING_CENTER         1320
#define STEERING_RIGHT_LIMIT    1850
#define STEERING_FINE_STEP      30

static const char *TAG = "RC_CTRL";

// MCPWM compare handles
static mcpwm_cmpr_handle_t esc_comparator = NULL;
static mcpwm_cmpr_handle_t servo_comparator = NULL;

// Current output values
static uint32_t throttle_us = THROTTLE_NEUTRAL;
static uint32_t steering_us = STEERING_CENTER;

static uint32_t clamp_u32(uint32_t value, uint32_t min_val, uint32_t max_val)
{
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

static void set_throttle_us(uint32_t pulse_us)
{
    throttle_us = clamp_u32(pulse_us, THROTTLE_REVERSE_LIMIT, THROTTLE_FORWARD_LIMIT);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_comparator, throttle_us));
    ESP_LOGI(TAG, "Throttle = %lu us", (unsigned long)throttle_us);
}

static void set_steering_us(uint32_t pulse_us)
{
    steering_us = clamp_u32(pulse_us, STEERING_LEFT_LIMIT, STEERING_RIGHT_LIMIT);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo_comparator, steering_us));
    ESP_LOGI(TAG, "Steering = %lu us", (unsigned long)steering_us);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Boot throttle = %d us", THROTTLE_NEUTRAL);
    ESP_LOGI(TAG, "Boot steering = %d us", STEERING_CENTER);

    // 1) Create timer
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .period_ticks = PERIOD_US,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // 2) Create operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // 3) Create comparators
    mcpwm_comparator_config_t cmp_config = {};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &esc_comparator));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &servo_comparator));

    // Set safe startup values BEFORE starting the timer
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc_comparator, THROTTLE_NEUTRAL));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo_comparator, STEERING_CENTER));

    // 4) Create generators
    mcpwm_gen_handle_t esc_generator = NULL;
    mcpwm_gen_handle_t servo_generator = NULL;

    mcpwm_generator_config_t esc_gen_config = {
        .gen_gpio_num = ESC_GPIO,
    };
    mcpwm_generator_config_t servo_gen_config = {
        .gen_gpio_num = SERVO_GPIO,
    };

    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &esc_gen_config, &esc_generator));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &servo_gen_config, &servo_generator));

    // 5) ESC waveform
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        esc_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        esc_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            esc_comparator,
            MCPWM_GEN_ACTION_LOW
        )
    ));

    // 6) Servo waveform
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        servo_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        servo_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            servo_comparator,
            MCPWM_GEN_ACTION_LOW
        )
    ));

    // 7) Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Apply startup state
    set_throttle_us(THROTTLE_NEUTRAL);
    set_steering_us(STEERING_CENTER);

    while (1) {
        int ch = getchar();

        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == 'w' || ch == 'W') {
            set_throttle_us(THROTTLE_FORWARD_SLOW);
        } else if (ch == 's' || ch == 'S') {
            set_throttle_us(THROTTLE_REVERSE_SLOW);
        } else if (ch == 'x' || ch == 'X') {
            set_throttle_us(THROTTLE_NEUTRAL);
        } else if (ch == 'a' || ch == 'A') {
            set_steering_us(STEERING_LEFT_LIMIT);
        } else if (ch == 'd' || ch == 'D') {
            set_steering_us(STEERING_RIGHT_LIMIT);
        } else if (ch == 'c' || ch == 'C') {
            set_steering_us(STEERING_CENTER);
        } else if (ch == 'i' || ch == 'I') {
            set_throttle_us(throttle_us + THROTTLE_FINE_STEP);
        } else if (ch == 'k' || ch == 'K') {
            set_throttle_us(throttle_us - THROTTLE_FINE_STEP);
        } else if (ch == 'j' || ch == 'J') {
            set_steering_us(steering_us - STEERING_FINE_STEP);
        } else if (ch == 'l' || ch == 'L') {
            set_steering_us(steering_us + STEERING_FINE_STEP);
        }
    }
}