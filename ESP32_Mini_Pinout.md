# ESP32 Mini (ESP32-S3FN8) Pinout and Interface Guide

Pinout of the ESP32 Mini board with a USB Type-C connector, as printed on the manufacturer's pinout guide. Pins are numbered top to bottom with the USB-C connector at the top.

## Specifications

| Item | Value |
|---|---|
| Chip | ESP32-S3FN8 |
| Flash | 8 MB (64 Mbit) |
| USB | Type-C (native USB: power, programming and debug) |
| Wi-Fi | 2.4 GHz 802.11 b/g/n |
| Bluetooth | 5.0 LE |
| Operating voltage | 5 V (USB) / 3.3 V |
| GPIOs | 18 (many support PWM, I²C, SPI, UART) |
| Size | 18 mm (W) x 28 mm (L) |

## Power pins

| Pin | Function |
|---|---|
| 5V | USB 5 V input / external 5 V input |
| 3V3 | 3.3 V output (max 500 mA) |
| GND | Ground |

## Left side (top to bottom)

| Pin | GPIO | Function | Group |
|---|---|---|---|
| 1 | GPIO43 | TX (TXD0) | UART |
| 2 | GPIO44 | RX (RXD0) | UART |
| 3 | GPIO0 | BOOT | Control |
| 4 | GPIO1 | GPIO (ADC1_CH0) | GPIO / ADC |
| 5 | GPIO2 | GPIO (ADC1_CH1) | GPIO / ADC |
| 6 | GPIO3 | GPIO | GPIO |
| 7 | GPIO4 | GPIO | GPIO |
| 8 | GPIO5 | GPIO | GPIO |
| 9 | GPIO6 | GPIO | GPIO |
| 10 | GPIO7 | GPIO | GPIO |
| 11 | GPIO8 | Onboard LED | Other |
| 12 | GND | Ground | GND |

## Right side (top to bottom)

| Pin | GPIO | Function | Group |
|---|---|---|---|
| 18 | 5V | 5 V power | Power |
| 17 | GND | Ground | GND |
| 16 | 3V3 | 3.3 V output | Power |
| 15 | GPIO13 | HSPI_MOSI | SPI |
| 14 | GPIO12 | HSPI_MISO | SPI |
| 13 | GPIO11 | HSPI_SCK | SPI |
| 12 | GPIO10 | HSPI_CS | SPI |
| 11 | GPIO9 | I2C_SDA | I²C |
| 10 | GPIO18 | I2C_SCL | I²C |
| 9 | GPIO48 | ADC1_CH5 | ADC |
| 8 | GPIO45 | ADC1_CH2 | ADC |
| 7 | GND | Ground | GND |

## Interfaces

| Interface | Signals |
|---|---|
| UART | TX = GPIO43 (TXD0), RX = GPIO44 (RXD0), hardware serial |
| I²C | SDA = GPIO9, SCL = GPIO18 (default I²C bus) |
| SPI (HSPI) | MOSI = GPIO13, MISO = GPIO12, SCK = GPIO11, CS = GPIO10 |
| ADC (12-bit) | GPIO1 (ADC1_CH0), GPIO2 (ADC1_CH1), GPIO45 (ADC1_CH2), GPIO48 (ADC1_CH5). Input range 0 to 3.3 V |
| PWM | Most GPIOs support PWM output (LEDC channels available) |

## Onboard LED

A WS2812 RGB LED (NeoPixel compatible) is connected to GPIO8. It needs a NeoPixel-style driver, not a plain `digitalWrite`.

## Buttons and download mode

| Button | Function |
|---|---|
| BOOT (GPIO0) | Hold BOOT while pressing RESET to enter download mode. Keeping BOOT pressed prevents a normal boot. |
| RESET | Restarts the board |

To enter download mode:
1. Hold BOOT.
2. Press RESET.
3. Release RESET.
4. Release BOOT.

The board is now in download mode, ready for flashing.

## Notes

- **Check the ADC list.** The guide lists GPIO45 and GPIO48 as ADC pins. In the ESP32-S3 datasheet the ADC1 channels are on GPIO1 to GPIO10, so GPIO45 and GPIO48 may not work as analog inputs. For analog reads, use GPIO1 to GPIO7, which are broken out on this board, and verify the others before relying on them.
- **GPIO0 and GPIO45 are strapping pins** on the ESP32-S3. Avoid pulling them to unexpected levels at boot.
- GPIO43 and GPIO44 are the default UART0 pins, so using them for other signals can interfere with serial output.
- The 3V3 pin can supply at most 500 mA. Power motors, servos and pumps from a separate supply.
