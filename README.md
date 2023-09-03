# EtherCan
## STM32F1 and Wiznet W5500 Chip, Ethernet to CAN-Bus Transceiver Firmware

## Table of Contents
- [Introduction](#Introduction)
- [Features](#Features)
- [Hardware Requirements](#Hardware-Requirements)
- [Getting Started](#Getting-Started)

# Introduction
This firmware project is designed for the STM32F1 microcontroller series and utilizes the Wiznet W5500 Ethernet chip. It provides a foundation for building network-enabled applications with these components. Whether you are working on IoT projects, web servers, or networked sensors, this firmware serves as a starting point for your development.

# Features
- Ethernet connectivity using the Wiznet W5500 chip.
- Example application showcasing network communication.
- Lightweight and modular design for easy customization.
- Compatibility with STM32F1 series microcontrollers.

# Requirements
To run this firmware, you will need the following hardware components:
- [EtherCan Hardware](https://github.com/kf1375/EtherCan_Hardware)
- 12V Power supply
- Appropriate connections between the EtherCan Hardware, the ethernet host and CAN-Bus nodes.

# Getting Started
- Clone or download this repository to your development environment.
- Ensure you have a compatible toolchain and development environment of STM32CubeIDE for STM32F1.
- Connect the STM32F1 through SWD interface to your host with ST-Link or J-Link.
- Open the project in STM32CubeIDE.
- Build and flash the firmware onto the [EtherCan](https://github.com/kf1375/EtherCan_Hardware) board.