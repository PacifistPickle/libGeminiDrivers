# libGeminiDrivers
A collection of simple C++ Drivers. Currently, there is only one driver for humidity... Gotta start somewhere I guess.

## Drivers
 - ST [HTS221]((./libHumidity/include/Hts221.h)) - Humidity and Temperature sensor I2C driver

## Feature Highlights
 - Driver is Hardware/Architecture agnostic
     - Driver code is pulled in via a git submodule to avoid dependencies
     - Currently compiles for Arm (Ras Pi) and x86 architecture
     - Should be easily expandable to new processors by adding a new toolchain file
 - Written in C++

## How Do I use this Library?
I tried hard to make the code self documenting.
 - See my [rasPiHts221Demo](https://github.com/gemini-dakota/rasPiHts221Demo) repository for how to compile and run this driver on the Raspberyr Pi
 - See [Hts221.h](./libHumidity/include/Hts221.h) header file for driver implementation details and generic usage examples. All functions are fully Doxygen commented
