/**
 * ATOM ECHO M5 - LED & Button Control Example
 * ============================================
 * This program demonstrates basic hardware control on the ESP32-based Atom Echo M5 device.
 * It shows how to read button input and control an RGB LED using the RMT peripheral.
 * 
 * Learning Resources:
 * - ESP32 Programming Guide: https://docs.espressif.com/projects/esp-idf/en/latest/
 * - FreeRTOS (the operating system): https://www.freertos.org/
 * - C Programming basics: https://www.learn-c.org/
 */

// === HEADER FILES (Libraries) ===
// In C, we use #include to bring in code from other files (like import in Python)
#include "freertos/FreeRTOS.h"      // FreeRTOS: Real-time operating system that manages tasks
#include "freertos/task.h"          // Task management functions (delays, scheduling)
#include "driver/gpio.h"            // GPIO: General Purpose Input/Output - controls pins
#include "driver/rmt_tx.h"          // RMT: Remote Control Transceiver - generates precise timing signals
#include "driver/rmt_encoder.h"     // Encoder functions to convert data for RMT transmission
#include "esp_log.h"                // Logging system for debug messages (like print() in Python)

// === CONSTANTS AND MACROS ===
// #define creates constants - these are replaced by the preprocessor before compilation
// Using constants makes code more readable and maintainable

// TAG is used for logging - helps identify which part of code generated a log message
static const char *TAG = "ATOM_ECHO";

// Hardware Pin Definitions
// The Atom Echo M5 has specific pins connected to the button and LED
#define BUTTON_PIN    39   // GPIO39 - Connected to the physical button
                          // Note: GPIO39 is "input only" - can't output signals
                          // Also lacks internal pull-up resistor capability

#define LED_PIN       27   // GPIO27 - Connected to the WS2812 RGB LED data input

// === WS2812 LED TIMING PARAMETERS ===
// The WS2812 is a "smart" LED that receives color data through precisely timed signals
// It uses a single wire protocol where timing determines if a bit is 0 or 1
// These times are in nanoseconds (ns) - billionths of a second!

#define WS2812_T0H_NS   350    // Time the signal is HIGH for a '0' bit (350 nanoseconds)
#define WS2812_T0L_NS   1000   // Time the signal is LOW for a '0' bit (1000 nanoseconds)
#define WS2812_T1H_NS   1000   // Time the signal is HIGH for a '1' bit (1000 nanoseconds)
#define WS2812_T1L_NS   350    // Time the signal is LOW for a '1' bit (350 nanoseconds)
#define WS2812_RESET_US 280    // Reset time between color updates (280 microseconds)

// RMT (Remote Control Transceiver) Configuration
// The RMT peripheral can generate very precise timing signals
// We set it to 10MHz = 10,000,000 Hz = 100ns per "tick"
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 

// === GLOBAL VARIABLES ===
// 'static' means these variables are only visible within this file (private)
// Global variables maintain their values throughout the program's execution

static bool led_on = false;  // Boolean flag: true = LED is on, false = LED is off
                             // 'bool' is a type that can only be true or false

// RMT Hardware Handles
// These are "handles" (like references or pointers) to hardware resources
// The ESP-IDF uses handles to manage hardware abstractly
static rmt_channel_handle_t led_chan = NULL;    // Handle for the RMT channel
static rmt_encoder_handle_t led_encoder = NULL; // Handle for the data encoder

// Color Data Buffer
// This array stores the RGB color values we want to send to the LED
// Array of 3 bytes: [Green, Red, Blue] - WS2812 expects GRB order, not RGB!
static uint8_t led_strip_pixels[3];  // uint8_t = unsigned 8-bit integer (0-255)

/**
 * Function: ws2812_set_color
 * ==========================
 * Sets the color of the WS2812 RGB LED
 * 
 * Parameters:
 * - red:   Red intensity (0-255, where 0 = off, 255 = maximum brightness)
 * - green: Green intensity (0-255)
 * - blue:  Blue intensity (0-255)
 * 
 * How it works:
 * 1. Stores color values in the correct order (GRB not RGB)
 * 2. Uses RMT to transmit the data with precise timing
 * 3. Waits for transmission to complete before returning
 * 
 * 'static' means this function is only visible within this file
 * 'void' means the function doesn't return any value
 */
static void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    // WS2812 expects data in GRB (Green-Red-Blue) format, not RGB
    // This is a quirk of the WS2812 protocol we must accommodate
    led_strip_pixels[0] = green;  // First byte sent is green
    led_strip_pixels[1] = red;    // Second byte sent is red
    led_strip_pixels[2] = blue;   // Third byte sent is blue
    
    // Configure the transmission
    // This structure tells the RMT how to send the data
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // Don't repeat the transmission (0 = send once)
                         // Setting this to 1+ would repeat the pattern
    };
    
    // Transmit the color data to the LED
    // rmt_transmit sends our 3 bytes through the RMT peripheral
    // The encoder knows how to convert each bit to the proper timing signals
    ESP_ERROR_CHECK(rmt_transmit(
        led_chan,                    // Which RMT channel to use
        led_encoder,                 // Which encoder to use for conversion
        led_strip_pixels,            // Pointer to our color data
        sizeof(led_strip_pixels),    // Size of data (3 bytes)
        &tx_config                   // Transmission configuration
    ));
    
    // Wait for the transmission to complete
    // This is important! The RMT works in the background (asynchronously)
    // We must wait for it to finish before sending new data
    // portMAX_DELAY means wait forever if necessary (blocking call)
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

/**
 * Function: app_main
 * ==================
 * The main entry point of the program (like main() in standard C)
 * ESP-IDF calls this function after system initialization
 * This function sets up our hardware and runs the main program loop
 * 
 * Note: Unlike standard C, we don't need "int main()" because
 * FreeRTOS handles the program lifecycle differently
 */
void app_main(void) {
    // === INITIALIZATION PHASE ===
    
    // Log an informational message
    // ESP_LOGI creates an "info" level log (there's also LOGW for warning, LOGE for error)
    // %d in the log string gets replaced with the pin numbers (decimal integers)
    ESP_LOGI(TAG, "Atom Echo M5 Button & LED Test");
    
    // === CONFIGURE THE BUTTON (GPIO INPUT) ===
    
    // Create a configuration structure for the GPIO
    // In C, structures group related variables together
    gpio_config_t button_config = {
        // .pin_bit_mask selects which pins to configure
        // (1ULL << BUTTON_PIN) creates a 64-bit number with only bit 39 set
        // ULL = Unsigned Long Long (64-bit)
        // << is the left shift operator: 1 << 39 moves the bit 1 to position 39
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        
        // Set the pin as input (reads external signals)
        // GPIO_MODE_INPUT means we'll read from this pin, not write to it
        .mode = GPIO_MODE_INPUT,
        
        // Pull-up resistor configuration
        // Pull-up resistors ensure the pin reads HIGH when nothing is connected
        // GPIO39 doesn't support internal pull-up (hardware limitation)
        // The Atom Echo has an external pull-up resistor on the board
        .pull_up_en = GPIO_PULLUP_DISABLE,
        
        // Pull-down resistor configuration
        // Pull-down resistors ensure the pin reads LOW when nothing is connected
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        
        // Interrupt configuration
        // Interrupts can trigger functions when pin state changes
        // We're not using interrupts, just polling the pin state
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    // Apply the configuration to the GPIO hardware
    gpio_config(&button_config);
    
    // === CONFIGURE THE RMT FOR WS2812 LED CONTROL ===
    
    // Configure the RMT TX (transmit) channel
    // RMT can generate complex waveforms with precise timing
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = LED_PIN,                      // Which GPIO pin to use for output
        .clk_src = RMT_CLK_SRC_DEFAULT,          // Clock source (uses APB clock, usually 80MHz)
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ, // 10MHz = 100ns resolution
        .mem_block_symbols = 64,                  // Memory for storing waveform patterns
                                                  // Each symbol = one high/low period
        .trans_queue_depth = 4,                   // Number of pending transactions
                                                  // Allows buffering multiple transmissions
    };
    
    // Create the RMT channel with our configuration
    // &led_chan passes the address where the handle will be stored
    // The & operator gets the address of a variable (pointer)
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    
    // === CONFIGURE THE ENCODER ===
    // The encoder converts our byte data into RMT symbols (timing patterns)
    
    rmt_bytes_encoder_config_t encoder_config = {
        // Define what a '0' bit looks like
        .bit0 = {
            .level0 = 1,                          // Start HIGH
            .duration0 = WS2812_T0H_NS / 100,    // HIGH for 350ns (3.5 ticks at 100ns/tick)
            .level1 = 0,                          // Then go LOW  
            .duration1 = WS2812_T0L_NS / 100,    // LOW for 1000ns (10 ticks)
        },
        // Define what a '1' bit looks like
        .bit1 = {
            .level0 = 1,                          // Start HIGH
            .duration0 = WS2812_T1H_NS / 100,    // HIGH for 1000ns (10 ticks)
            .level1 = 0,                          // Then go LOW
            .duration1 = WS2812_T1L_NS / 100,    // LOW for 350ns (3.5 ticks)
        },
        // Configuration flags
        .flags.msb_first = 1,                    // Send Most Significant Bit first
                                                 // MSB first means bit 7 sent before bit 0
    };
    
    // Create the encoder with our configuration
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&encoder_config, &led_encoder));
    
    // Enable the RMT channel (turn it on)
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    
    // === INITIAL LED STATE ===
    
    // Turn off the LED initially (set all colors to 0)
    ws2812_set_color(0, 0, 0);
    
    // Small delay to ensure the LED receives the OFF command
    // pdMS_TO_TICKS converts milliseconds to FreeRTOS ticks
    // vTaskDelay pauses the current task (non-blocking for other tasks)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // === BUTTON DEBOUNCING VARIABLES ===
    // Mechanical buttons "bounce" - they make/break contact multiple times when pressed
    // Debouncing ensures we only register one press per physical button push
    
    bool last_button_state = true;     // Previous button reading (HIGH when not pressed)
    bool current_button_state = true;  // Current button reading
    TickType_t last_press_time = 0;    // Timestamp of last valid press
                                       // TickType_t is FreeRTOS's time unit
    
    // Debounce delay - ignore button changes for 50ms after a press
    const TickType_t debounce_delay = pdMS_TO_TICKS(50);
    
    // Log that we're ready
    ESP_LOGI(TAG, "Ready! Press the button to toggle LED");
    ESP_LOGI(TAG, "Button GPIO: %d, LED GPIO: %d", BUTTON_PIN, LED_PIN);
    
    // === MAIN PROGRAM LOOP ===
    // This loop runs forever (embedded systems typically never "exit")
    
    while (1) {  // while(1) creates an infinite loop
        // Read the current button state
        // gpio_get_level returns 0 (LOW) or 1 (HIGH)
        // Button pressed = LOW (0), Button released = HIGH (1)
        current_button_state = gpio_get_level(BUTTON_PIN);
        
        // Get the current system time (in ticks)
        // This is used for debouncing timing
        TickType_t current_time = xTaskGetTickCount();
        
        // Detect button press (HIGH to LOW transition)
        // This is "edge detection" - we detect the moment of change
        if (current_button_state == 0 && last_button_state == 1) {
            // Check if enough time has passed since last press (debouncing)
            if ((current_time - last_press_time) > debounce_delay) {
                // Valid button press detected!
                
                // Toggle the LED state using the NOT operator (!)
                // If led_on is true, !led_on is false, and vice versa
                led_on = !led_on;
                
                ESP_LOGI(TAG, "Button pressed");
                
                // Use conditional statement to set LED color based on state
                if (led_on) {
                    // Turn on the LED with white color
                    // Using moderate brightness (50 out of 255) to avoid eye strain
                    // White = equal amounts of Red, Green, and Blue
                    ws2812_set_color(50, 50, 50);
                    ESP_LOGI(TAG, "LED ON");
                } else {
                    // Turn off the LED (all colors = 0)
                    ws2812_set_color(0, 0, 0);
                    ESP_LOGI(TAG, "LED OFF");
                }
                
                // Record the time of this button press
                last_press_time = current_time;
            }
        }
        
        // Save current state for next iteration
        // This allows us to detect state changes (transitions)
        last_button_state = current_button_state;
        
        // Small delay to prevent CPU hogging
        // Without this delay, the loop would run thousands of times per second
        // 10ms delay = 100 checks per second (more than enough for button detection)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Note: We never reach here because of the infinite loop
    // In embedded systems, the program runs until power is removed
}