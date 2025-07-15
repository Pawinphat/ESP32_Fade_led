#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";
static const char *TAG_LED = "Led status";
static const char *TAG_Fade = "fade_led";

#define BLINK_GPIO CONFIG_BLINK_GPIO

// PWM Configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (2) // GPIO2 - your LED pin
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX (8191) // Maximum duty cycle value (2^13 - 1)
#define LEDC_FREQUENCY (5000) // Frequency in Hertz. Set frequency at 5 kHz



// Fade parameters
#define FADE_TIME (1000) // Fade time in milliseconds
#define FADE_STEP_TIME (10)	// Time between fade steps in milliseconds



static uint8_t s_led_state = 0;



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
		.speed_mode= LEDC_MODE,
		.timer_num= LEDC_TIMER,
		.duty_resolution= LEDC_DUTY_RES,
		.freq_hz= LEDC_FREQUENCY,
		.clk_cfg= LEDC_AUTO_CLK
	};

	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	// Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel = {
		.speed_mode= LEDC_MODE,
		.channel= LEDC_CHANNEL,
		.timer_sel= LEDC_TIMER,
		.intr_type= LEDC_INTR_DISABLE,
		.gpio_num= LEDC_OUTPUT_IO,
		.duty= 0, // Set duty to 0%
		.hpoint= 0
	};
	
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	ESP_LOGI(TAG, "LED PWM configured on GPIO %d", LEDC_OUTPUT_IO);

}



static void fade_led_up(void)
{

	ESP_LOGI(TAG, "Fading LED UP");

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

// Choose one of the fade methods:
// Method 1: Manual fade (more control)

while (1) {
	fade_led_up();
	vTaskDelay(500 / portTICK_PERIOD_MS); // Stay bright for 500ms
	fade_led_down();
	vTaskDelay(500 / portTICK_PERIOD_MS); // Stay off for 500ms

	}
}