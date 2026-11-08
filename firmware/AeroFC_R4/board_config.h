// board_config.h - wiring of the Arduino UNO R4 Minima.  Change pins here if you wire differently.
//
//   OLED 0.96" SSD1306 (I2C, 4 pins):  GND -> GND,  VCC -> 5V,  SCL -> A5 (or SCL pin),  SDA -> A4 (or SDA pin)
//   Buttons (each between the pin and GND, no resistor needed - internal pull-ups are used):
//       UP -> D2     DOWN -> D3     SELECT -> D4     BACK -> D5
//   Optional telemetry for a USB-TTL adapter: R4 pin D1 (TX) -> adapter RX,  GND -> GND   (460800 baud)
//   USB-C: the simulator (HIL protocol, any baud rate - it is native USB)
#pragma once

#define PIN_BTN_UP      2
#define PIN_BTN_DOWN    3
#define PIN_BTN_SELECT  4
#define PIN_BTN_BACK    5
#define PIN_STATUS_LED  LED_BUILTIN   // D13, the yellow "L" LED on the board

#define OLED_I2C_CLOCK  400000        // 400 kHz fast mode
#define TELEMETRY_BAUD  460800        // Serial1 (D1 TX), USB-TTL adapters handle it fine
#define WATCHDOG_MS     1000          // armed only: board resets itself if the program ever hangs for 1 s
#define UI_FRAME_MS     200           // OLED refresh (5 Hz)
