/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: LicenseRef-Included
 */

#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"

static led_strip_handle_t s_led_strip;
static uint8_t s_red = 255, s_green = 255, s_blue = 255;
static uint8_t s_brightness = 255;
static bool s_power = false;

static void update_led(void)
{
    if (s_power) {
        // Застосовуємо яскравість до кожного кольору
        uint8_t r = (s_red * s_brightness) / 255;
        uint8_t g = (s_green * s_brightness) / 255;
        uint8_t b = (s_blue * s_brightness) / 255;
        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, r, g, b));
    } else {
        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, 0, 0, 0));
    }
    ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

void light_driver_set_power(bool power)
{
    s_power = power;
    update_led();
}

void light_driver_set_color_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    s_red = red;
    s_green = green;
    s_blue = blue;
    update_led();
}

void light_driver_set_level(uint8_t level)
{
    s_brightness = level;
    update_led();
}

void light_driver_init(bool power)
{
    led_strip_config_t led_strip_conf = {
        .max_leds = CONFIG_EXAMPLE_STRIP_LED_NUMBER,
        .strip_gpio_num = CONFIG_EXAMPLE_STRIP_LED_GPIO,
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));
    
    light_driver_set_power(power);
}