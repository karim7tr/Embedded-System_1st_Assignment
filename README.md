# Embedded-System_1st_Assignment

This application is designed for an embedded system, utilizing PIC24 microcontroller, to manage UART communication, SPI interface, and timer-based interrupt handling.

## Overview

The code consists of functionalities for handling interrupts, managing circular buffers for UART communication, and controlling SPI-based LCD display. It's organized into sections for initialization, interrupt handlers, timer configurations, communication functions, and the main loop.

## Features

- **Interrupt Handling:** 
    - Utilizes Timer1 and Timer2 interrupts to manage specific functionalities like LED control and button debouncing.
    - UART RX interrupt is used to handle received data.
- **Communication Interfaces:**
    - SPI1 is configured for communication with an LCD display.
    - UART2 is set up for serial communication.
- **Buffer Management:**
    - Circular buffer structure is implemented for UART data reception.

## Usage

### Setup
1. Configure the PIC24 microcontroller with the provided configuration settings.
2. Initialize SPI and UART interfaces using `initSPI()` and `initUART()` functions.
3. Set up circular buffer for UART communication via `initCircularBuffer()`.

### Main Functionality
- The `main()` function orchestrates the entire application, implementing an algorithm, managing UART buffer, and handling interrupts.
- The program displays characters received through UART on an SPI-driven LCD screen.
- It reacts to button presses (S5 and S6), triggering specific actions like LED control, data transmission, and LCD display manipulation.

### Important Functions
- `addToBuffer(char data)`: Adds received UART characters to a circular buffer for processing.
- `sendSPI(char data)`: Sends data through SPI to control the LCD display.
- `sendUARTBuffer(char buffer[])`: Transmits data via UART.
- Interrupt handlers (`_T1Interrupt()`, `_T2Interrupt()`, `_U2RXInterrupt()`) manage specific interrupt-driven tasks.

## Notes

- Adjust timer settings (`tmr_setup_period()`, `tmr_wait_ms()`) based on required time intervals.
- Ensure proper handling of UART buffer overflows (`addToBuffer()` function).

---

- Karim Triki S5528602
- Roumaissa Benkredda S5434673
