# Atom Echo M5 - Button & LED Control Example

A beginner-friendly ESP32 project demonstrating hardware control on the Atom Echo M5 device. This example shows how to read button input and control an RGB LED using modern ESP-IDF APIs.

## 📚 What You'll Learn

This project is perfect for beginners learning embedded C programming and ESP32 development:

- **C Programming Basics**: Variables, functions, structures, pointers, and control flow
- **Hardware Control**: GPIO input/output, reading buttons, controlling LEDs
- **ESP32 Peripherals**: Using the RMT (Remote Control Transceiver) for precise timing
- **Real-time Systems**: Understanding FreeRTOS tasks and timing
- **Embedded Concepts**: Debouncing, edge detection, infinite loops
- **Protocol Implementation**: WS2812 "NeoPixel" LED communication

## 🎯 Project Overview

### What It Does
- Reads button presses from the built-in button
- Toggles an RGB LED on/off with each button press
- Implements proper button debouncing for reliable operation
- Uses the modern ESP-IDF v5.x RMT driver API (no deprecation warnings!)

### Hardware Used
- **Atom Echo M5**: A compact ESP32-PICO based development board
- **Built-in Button**: Connected to GPIO39 (input-only pin)
- **WS2812 RGB LED**: Connected to GPIO27 (addressable "smart" LED)

## 🛠️ Development Setup

### Prerequisites
1. **ESP-IDF v5.0+**: The official ESP32 development framework
   ```bash
   # Install ESP-IDF following the official guide:
   # https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
   ```

2. **USB Driver**: For the CH9102 USB-to-Serial chip on Atom Echo
   - Most systems have this built-in
   - If needed: [CH9102 Driver](http://www.wch-ic.com/downloads/CH341SER_ZIP.html)

### Project Structure
```
atom_echo_beep/
├── main/
│   ├── CMakeLists.txt    # Build configuration
│   └── main.c            # Main program (heavily commented!)
├── CMakeLists.txt        # Project configuration
├── README.md            # This file
└── sdkconfig            # ESP-IDF configuration
```

## 🚀 Quick Start

### Building and Flashing

```bash
# Clean any previous builds
idf.py fullclean

# Set target to ESP32
idf.py set-target esp32

# Build the project
idf.py build

# Find your serial port
ls /dev/cu.*
# Look for something like: /dev/cu.usbserial-XXXXXXXX

# Flash to device (use slower baud rate for reliability)
idf.py -p /dev/cu.usbserial-5552A2D146 -b 115200 flash

# Monitor serial output
idf.py -p /dev/cu.usbserial-5552A2D146 monitor
# Press Ctrl+] to exit monitor
```

### Troubleshooting

If you have issues flashing, try:

```bash
# Erase flash completely and start fresh
python -m esptool --port /dev/cu.usbserial-5552A2D146 --chip esp32 erase_flash

# Then try flashing again with even slower baud rate
idf.py -p /dev/cu.usbserial-5552A2D146 -b 9600 flash
```

## 📖 Understanding the Code

The code is extensively commented to help beginners understand every concept. Here are the key sections:

### 1. **Header Files & Includes**
```c
#include "freertos/FreeRTOS.h"  // Real-time operating system
#include "driver/gpio.h"        // GPIO control
#include "driver/rmt_tx.h"      // RMT transmitter (for LED timing)
```

### 2. **GPIO Configuration**
- **Input Pin (Button)**: GPIO39 - special "input-only" pin
- **Output Pin (LED)**: GPIO27 - controls WS2812 RGB LED

### 3. **WS2812 LED Protocol**
The WS2812 is a "smart" LED that receives data through precise timing:
- **Bit 0**: 350ns HIGH + 1000ns LOW
- **Bit 1**: 1000ns HIGH + 350ns LOW
- **Data Format**: 24 bits total (8 bits each for Green, Red, Blue)

### 4. **Button Debouncing**
Mechanical buttons "bounce" - they make/break contact multiple times when pressed. The code implements a 50ms debounce delay to ensure clean button detection.

### 5. **Main Loop**
The program runs in an infinite loop (common in embedded systems):
1. Read button state
2. Detect press (HIGH→LOW transition)
3. Toggle LED if valid press detected
4. Small delay to prevent CPU hogging

## 🔍 Key Concepts Explained

### Pointers and Addresses (`&` operator)
```c
ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
//                                  ^                ^
//                            Address of config   Address to store handle
```
The `&` gets the memory address of a variable, allowing functions to modify it.

### Bit Manipulation (`<<` operator)
```c
.pin_bit_mask = (1ULL << BUTTON_PIN)  // Creates: 0x8000000000
//               ^    ^
//               1    Shift left by 39 positions
```
Used to select specific GPIO pins by setting individual bits.

### Structure Initialization
```c
gpio_config_t button_config = {
    .pin_bit_mask = ...,    // Named field initialization
    .mode = GPIO_MODE_INPUT,
    // ...
};
```
C structures group related data; we initialize them with named fields.

### Handle-based APIs
Modern ESP-IDF uses "handles" (opaque pointers) to manage hardware:
```c
rmt_channel_handle_t led_chan;  // Handle to RMT channel
rmt_encoder_handle_t led_encoder;  // Handle to encoder
```

## 📚 Learning Resources

### For C Programming Beginners
- **Interactive Tutorial**: [Learn-C.org](https://www.learn-c.org/)
- **Book**: "The C Programming Language" by K&R (classic)
- **Video Series**: CS50 Introduction to Computer Science

### For ESP32 Development
- **Official Docs**: [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- **Examples**: [ESP-IDF Examples Repository](https://github.com/espressif/esp-idf/tree/master/examples)
- **Forum**: [ESP32 Forum](https://www.esp32.com/)

### Understanding Concepts
- **FreeRTOS**: [FreeRTOS.org](https://www.freertos.org/) - Real-time operating system
- **GPIO**: General Purpose Input/Output - digital pin control
- **RMT**: Remote Control Transceiver - generates precise waveforms
- **WS2812**: [Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812.pdf) - LED protocol details

## 🎨 Customization Ideas

Once you understand the basic code, try these modifications:

1. **Change LED Colors**: Modify the RGB values in `ws2812_set_color()`
   ```c
   ws2812_set_color(255, 0, 0);  // Bright red
   ws2812_set_color(0, 255, 0);  // Bright green
   ws2812_set_color(0, 0, 255);  // Bright blue
   ```

2. **Add Color Cycling**: Make the LED cycle through colors
3. **Pattern Sequences**: Create blinking or fading patterns
4. **Multiple Press Actions**: Different actions for short/long press
5. **Rainbow Effect**: Smoothly transition through color spectrum

## 🐛 Common Issues & Solutions

### LED Doesn't Light Up
- Check GPIO27 connection
- Verify WS2812 timing parameters
- Ensure proper power supply (3.3V)

### Button Doesn't Respond
- GPIO39 has no internal pull-up (external resistor on board)
- Check debounce timing
- Verify button is connected to GPIO39

### Build Warnings
- This code uses the modern RMT API (driver/rmt_tx.h)
- No deprecation warnings with ESP-IDF v5.0+

## 📄 License

This example code is provided as-is for educational purposes. Feel free to use and modify for your projects!

## 🤝 Contributing

Found an issue or have an improvement? Feel free to contribute!
1. Understand the existing code structure
2. Keep comments beginner-friendly
3. Test your changes on actual hardware

---

**Happy Learning!** 🎉 This project is designed to help you understand embedded C programming and ESP32 development. Take your time to read through the comments in `main.c` - they explain every concept in detail!