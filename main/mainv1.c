//Control Throttle using "W" and "S". "X" goes back to neutral
//Slow forward start at 1580
//Regular forward speed at 1600
//Slow Reverse start at 1410
//idf.py -p COM5 flash monitor
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

#define ESC_GPIO             17

#define TIMER_RESOLUTION_HZ  1000000   // 1 tick = 1 us
#define PERIOD_US            20000     // 20 ms = 50 Hz

#define THROTTLE_MIN_US      1000
#define THROTTLE_NEUTRAL_US  1500
#define THROTTLE_MAX_US      2000
#define THROTTLE_STEP_US     10

static const char *TAG = "ESC";

static mcpwm_cmpr_handle_t comparator = NULL;
static uint32_t current_throttle_us = THROTTLE_NEUTRAL_US;

static uint32_t clamp_throttle(uint32_t value)
{
    if (value < THROTTLE_MIN_US) {
        return THROTTLE_MIN_US;
    }
    if (value > THROTTLE_MAX_US) {
        return THROTTLE_MAX_US;
    }
    return value;
}

static void set_throttle_us(uint32_t pulse_us)
{
    current_throttle_us = clamp_throttle(pulse_us);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, current_throttle_us));
    ESP_LOGI(TAG, "Throttle = %lu us", (unsigned long)current_throttle_us);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESC PWM");
    ESP_LOGI(TAG, "Booting safely at %d us", THROTTLE_NEUTRAL_US);

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

    // 3) Create comparator
    mcpwm_comparator_config_t cmp_config = {};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &comparator));

    // CRITICAL: set neutral BEFORE starting the timer
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, THROTTLE_NEUTRAL_US));

    // 4) Create generator on GPIO 17
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = ESC_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &generator));

    // 5) PWM actions:
    // HIGH at the start of the cycle, LOW when compare matches
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            comparator,
            MCPWM_GEN_ACTION_LOW
        )
    ));

    // 6) Start PWM
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Hold neutral initially
    set_throttle_us(THROTTLE_NEUTRAL_US);

    ESP_LOGI(TAG, "Controls:");
    ESP_LOGI(TAG, "  w = +10 us");
    ESP_LOGI(TAG, "  s = -10 us");
    ESP_LOGI(TAG, "  x = neutral (1500 us)");
    ESP_LOGI(TAG, "Always power the ESC on with throttle at 1500 us.");

    while (1) {
        int ch = getchar();

        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (ch == 'w' || ch == 'W') {
            set_throttle_us(current_throttle_us + THROTTLE_STEP_US);
        } else if (ch == 's' || ch == 'S') {
            set_throttle_us(current_throttle_us - THROTTLE_STEP_US);
        } else if (ch == 'x' || ch == 'X') {
            set_throttle_us(THROTTLE_NEUTRAL_US);
        }
    }
}