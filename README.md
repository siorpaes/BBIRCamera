# Bare bones Thermal Camera
This project implements a very simple thermal camera based on STM32F401 microcontroller, Panasonic AMG8833 IR sensor and ILI9341 TFT display. The project can be easily adapted to other STMicroelectronics families as Cube HAL drivers are used for ease of porting.

Hardware shopping list:

* [NUCLEO-F401RE board](https://www.amazon.com/Waveshare-NUCLEO-F401RE-Development-STM32F401RE-Programmer/dp/B00N1RMN36/ref=sr_1_1?ie=UTF8&qid=1529413822&sr=8-1&keywords=nucleo-f401re)
* [Adafruit AMG8833 IR Thermal Camera Breakout](https://www.adafruit.com/product/3538)
* [2.8" TFT Display with Resistive Touchscreen](https://www.adafruit.com/product/1774)


Pinout for NUCLEO-F401RE board


|Signal        | STM32 IO | Nucleo connector |
|--------------|:--------:|:----------------:|
| AMG8833 SCL1 |   PB8    |      CN10-3      |
| AMG8833 SDA1 |   PB9    |      CN10-5      |
| AMG8833 INT  |   PC8    |      CN10-2      |
| ILI9341 CLK  |   PB10   |      CN10-25     |
| ILI9341 MOSI |   PC3    |      CN7-37      |
| ILI9341 MISO |   PC2    |      CN7-35      |
| ILI9341 CS   |   PB12   |      CN10-16     |
| ILI9341 RST  |   PB13   |      CN10-30     |
| ILI9341 D/C  |   PB14   |      CN10-28     |

Important note: as AMG8833 sensor I2C address is configurable using dedicated input, make sure that it matches with the address defined in driver's header file.
