#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
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

// LED state
static bool led_on = false;
static rmt_item32_t led_data[24];  // 24 bits for single RGB LED

// Convert nanoseconds to RMT ticks
static uint32_t ns_to_ticks(uint32_t ns) {
    return (ns / 12.5);  // 80MHz clock = 12.5ns per tick
}

// Set RGB color for WS2812
static void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t color = (green << 16) | (red << 8) | blue;  // GRB format for WS2812
    
    for (int i = 0; i < 24; i++) {
        if (color & (1 << (23 - i))) {
            // Send 1
            led_data[i].level0 = 1;
            led_data[i].duration0 = ns_to_ticks(WS2812_T1H_NS);
            led_data[i].level1 = 0;
            led_data[i].duration1 = ns_to_ticks(WS2812_T1L_NS);
        } else {
            // Send 0
            led_data[i].level0 = 1;
            led_data[i].duration0 = ns_to_ticks(WS2812_T0H_NS);
            led_data[i].level1 = 0;
            led_data[i].duration1 = ns_to_ticks(WS2812_T0L_NS);
        }
    }
    
    rmt_write_items(RMT_CHANNEL_0, led_data, 24, true);
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
    
    // Configure RMT for WS2812 LED
    rmt_config_t rmt_cfg = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL_0,
        .gpio_num = LED_PIN,
        .clk_div = 1,  // 80MHz clock
        .mem_block_num = 1,
        .tx_config = {
            .loop_en = false,
            .carrier_freq_hz = 0,
            .carrier_duty_percent = 0,
            .carrier_level = RMT_CARRIER_LEVEL_LOW,
            .carrier_en = false,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .idle_output_en = true,
        }
    };
    
    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL_0, 0, 0));
    
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