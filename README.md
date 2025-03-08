# STM32F4 Self-Balancing Robot Firmware

A layered firmware architecture for a self-balancing robot based on STM32F401RC microcontroller.

## Overview

This project implements a modular firmware architecture for a self-balancing robot, featuring:

- Hardware Abstraction Layer (HAL) for STM32F4 peripherals
- Device Driver Layer for sensors and actuators
- Middleware Layer for system services and signal processing
- Application Layer for robot logic and control algorithms

## Architecture

The firmware is structured with clean separation of concerns:

1. **Hardware Abstraction Layer**: Direct peripheral access with consistent API
   - GPIO, I2C, SPI, UART, Timers, DMA, etc.

2. **Device Driver Layer**: Hardware-independent device interfaces
   - Sensors (IMU, etc.)
   - Actuators (Motors, etc.)
   - Communication protocols

3. **Middleware Layer**: 
   - System services (delays, initialization)
   - Signal processing (filters, controllers)

4. **Application Layer**:
   - Robot logic (balancing algorithms)
   - System state management
   - Command processing

## Features

- Minimal CMSIS-based implementation
- No dependency on vendor HAL libraries
- Consistent error handling across layers
- Clean interface abstraction between layers
- Modular and testable components

## Hardware Support

- STM32F401RC microcontroller
- MPU6050 IMU (planned)
- Motor drivers (planned)
- RC receiver interface (planned)

## Getting Started

### Prerequisites

- ARM GCC toolchain
- Make
- ST-Link utility for flashing

### Building

```bash
make