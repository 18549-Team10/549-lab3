# Description

This directory contains the Arduino bootloader for Atmega MCUs.
The directory was pulled from [Arduino/hardware/arduino/avr/bootloaders/atmega]
(https://github.com/arduino/Arduino/tree/master/hardware/arduino/avr/bootloaders)
of the main https://github.com/arduino/Arduino repository.

Slight modifications have been made to the Makefile to set the default programmer
to the atmelice_isp and setting lock and efuse bits to a working value.
