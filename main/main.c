/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "esp_intr_alloc.h" // Defines ESP_INTR_FLAG_DEFAULT

static const char *TAG = "example";
static const char *TAG_LED = "Led status";
static const char *TAG_Fade = "fade_led";
static const char *TAG_BUTTON = "BUTTON_ISR";


/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BUTTON_GPIO CONFIG_BUTTON_GPIO

// PWM Configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (2) // GPIO2 - your LED pin
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX           (8191) // Maximum duty cycle value (2^13 - 1)
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

// Fade parameters
#define FADE_TIME               (1000) // Fade time in milliseconds
#define FADE_STEP_TIME          (10)   // Time between fade steps in milliseconds

static uint8_t s_led_state = 0;

// Global variable to track LED state (on/off)
static volatile bool led_is_on = false;
// Global variable to signal button press
static volatile bool button_pressed = false;

// ISR for button press
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    button_pressed = true;
}

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_led_pwm(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "LED PWM configured on GPIO %d", LEDC_OUTPUT_IO);
}


static void configure_button_sw(void)
{
    // Configure Button GPIO
    gpio_config_t io_conf = {};
    // Interrupt on any edge (for push and release) or just falling edge for a button press
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger on falling edge (button press)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up resistor
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);
    // Hook ISR handler for specific GPIO pin
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);
}


#else
#error "unsupported LED type"
#endif

static void fade_led_up(void)
{
    ESP_LOGI(TAG_Fade, "Fading LED UP");
    
    // Calculate fade steps
    int fade_steps = FADE_TIME / FADE_STEP_TIME;
    int duty_step = LEDC_DUTY_MAX / fade_steps;
    
    for (int duty = 0; duty <= LEDC_DUTY_MAX; duty += duty_step) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(FADE_STEP_TIME / portTICK_PERIOD_MS);
    }
    
    // Ensure we reach maximum brightness
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MAX);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}


static void fade_led_down(void)
{
    ESP_LOGI(TAG, "Fading LED DOWN");
    
    // Calculate fade steps
    int fade_steps = FADE_TIME / FADE_STEP_TIME;
    int duty_step = LEDC_DUTY_MAX / fade_steps;
    
    for (int duty = LEDC_DUTY_MAX; duty >= 0; duty -= duty_step) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(FADE_STEP_TIME / portTICK_PERIOD_MS);
    }
    
    // Ensure we reach minimum brightness (off)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Alternative: Hardware fade (smoother but less control)
static void hardware_fade_led(void)
{
    ESP_LOGI(TAG, "Hardware Fade UP");
    ledc_fade_func_install(0); // Install fade function
    
    // Fade up
    ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MAX, FADE_TIME);
    ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Hardware Fade DOWN");
    // Fade down
    ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, FADE_TIME);
    ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
    vTaskDelay(FADE_TIME / portTICK_PERIOD_MS);
}

void app_main(void)
{
	  // Configure LED PWM
    configure_led_pwm();
    configure_button_sw();
    
    // Initial state: LED off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI(TAG_Fade, "Initial state: LED is OFF.");    
    

    while (1) {
        if (button_pressed) {
            // Debounce delay (adjust as needed)
            vTaskDelay(pdMS_TO_TICKS(50));
            // Read button state again after debounce
            if (gpio_get_level(BUTTON_GPIO) == 0) { // Confirm button is still pressed (low)
                button_pressed = false; // Reset the flag immediately to avoid multiple triggers

                if (!led_is_on) {
                    // Turn LED ON (fade up)
                    fade_led_up();
                    led_is_on = true;
                    ESP_LOGI(TAG_Fade, "Button pressed: LED Fading UP.");
                } else {
                    // Turn LED OFF (fade down)
                    fade_led_down();
                    led_is_on = false;
                    ESP_LOGI(TAG_Fade, "Button pressed: LED Fading DOWN.");
                }
            } else {
                 button_pressed = false; // False positive, reset
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to prevent busy-waiting
    }


    // Configure the peripheral according to the LED type
    /*
    configure_led();

    while (1) {
        //ESP_LOGI(TAG_LED, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        ESP_LOGI(TAG_LED, "%s", s_led_state == true ? "ON" : "OFF");
        blink_led();
        // Toggle the LED state 
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
    
    */
}
