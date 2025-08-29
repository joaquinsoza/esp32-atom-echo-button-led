#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

static const char *TAG = "ATOM_ECHO";

// Atom Echo M5 hardware pins
#define BUTTON_PIN    39   // Button GPIO (input only, no internal pullup)
#define LED_PIN       27   // RGB LED data pin (WS2812)

// WS2812 timing parameters (in RMT ticks, 1 tick = 12.5ns at 80MHz)
#define WS2812_T0H_NS   350
#define WS2812_T0L_NS   1000
#define WS2812_T1H_NS   1000
#define WS2812_T1L_NS   350
#define WS2812_RESET_US 280

// LED state and RMT handles
static bool led_on = false;
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static uint8_t led_strip_pixels[3];  // RGB data buffer

// WS2812 timing parameters in nanoseconds
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 100ns per tick

// Set RGB color for WS2812
static void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    // WS2812 expects GRB format
    led_strip_pixels[0] = green;
    led_strip_pixels[1] = red;
    led_strip_pixels[2] = blue;
    
    // Transmit the color data
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    // Wait for transmission to complete
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

void app_main(void) {
    ESP_LOGI(TAG, "Atom Echo M5 Button & LED Test");
    
    // Configure button GPIO (GPIO39 is input-only, no internal pullup available)
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // GPIO39 doesn't support internal pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);
    
    // Configure RMT TX channel for WS2812 LED
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT, // use APB clock source
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64, // memory block size
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    
    // Configure bytes encoder for WS2812
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = WS2812_T0H_NS / 100,  // 100ns per tick at 10MHz
            .level1 = 0,
            .duration1 = WS2812_T0L_NS / 100,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = WS2812_T1H_NS / 100,
            .level1 = 0,
            .duration1 = WS2812_T1L_NS / 100,
        },
        .flags.msb_first = 1, // WS2812 expects MSB first
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&encoder_config, &led_encoder));
    
    // Enable RMT TX channel
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    
    // Turn off LED initially
    ws2812_set_color(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Variables for button debouncing
    bool last_button_state = true;  // Button is normally high (has external pullup)
    bool current_button_state = true;
    TickType_t last_press_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);
    
    ESP_LOGI(TAG, "Ready! Press the button to toggle LED");
    ESP_LOGI(TAG, "Button GPIO: %d, LED GPIO: %d", BUTTON_PIN, LED_PIN);
    
    while (1) {
        current_button_state = gpio_get_level(BUTTON_PIN);
        TickType_t current_time = xTaskGetTickCount();
        
        // Check for button press (button goes LOW when pressed)
        if (current_button_state == 0 && last_button_state == 1) {
            if ((current_time - last_press_time) > debounce_delay) {
                // Button was pressed
                led_on = !led_on;
                
                ESP_LOGI(TAG, "Button pressed");
                
                if (led_on) {
                    // Set LED to white (you can change RGB values)
                    ws2812_set_color(50, 50, 50);  // Moderate brightness white
                    ESP_LOGI(TAG, "LED ON");
                } else {
                    // Turn off LED
                    ws2812_set_color(0, 0, 0);
                    ESP_LOGI(TAG, "LED OFF");
                }
                
                last_press_time = current_time;
            }
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent CPU hogging
    }
}