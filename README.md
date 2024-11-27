# STM32_407_STOP_WATCH_BARE_METAL

## 📋 Project Description

This project implements a **Stopwatch** on the STM32F407 microcontroller using bare-metal programming. The stopwatch displays elapsed time on a 4-digit 7-segment display, showing seconds and milliseconds. The project makes use of **GPIO**, **Timers**, and **Interrupts** for real-time operations. The stopwatch is controlled by two buttons: one to start/stop the timer and another to reset it.

### Key Features:
- **Start/Stop Functionality**: Toggle the stopwatch state.
- **Reset Functionality**: Reset the stopwatch to zero.
- **Timer Interrupts**: Use STM32 Timer interrupts to track time with precision.
- **Seven-Segment Display**: Display time on a 4-digit 7-segment display.

This project demonstrates low-level programming without the use of an operating system (bare metal), relying on the STM32 hardware timers and GPIOs for all functionalities.

## 🔧 Hardware Setup

- **Microcontroller**: STM32F407
- **Display**: 4-digit 7-segment display
- **Buttons**:
  - **PC13**: Reset button
  - **PC14**: Start/Stop button

## 🖥️ Software Overview

The software implements the core functionality of the stopwatch:
1. **Timer Initialization**: The timer is configured to trigger interrupts every 1 ms.
2. **Button Inputs**: The two buttons (start/stop and reset) are read using GPIO inputs.
3. **Interrupts**: The timer interrupt service routine updates the stopwatch every millisecond.
4. **7-Segment Display**: The 4-digit 7-segment display is updated based on the current time, showing the integer part (seconds) and decimal part (milliseconds).

## 🕹️ Timer Interrupt Functionality

The timer is configured to generate an interrupt every 1 ms. When the interrupt occurs, the time is updated by increasing the millisecond counter (`decimal_counter`). Once the counter reaches 100, the second counter (`integer_counter`) is incremented by one, and the millisecond counter is reset. The system is designed to handle up to 60 seconds before it resets, ensuring the stopwatch can be used for extended periods.

## 📹 Demo Video

Check out the demo video for the **STM32 Stopwatch** in action:

[![STM32 Stopwatch Demo](https://img.youtube.com/vi/9SXp1Skzrg8/0.jpg)](https://www.youtube.com/watch?v=9SXp1Skzrg8)

## 📂 Project Files

The project includes the following files:
- **main.c**: Core logic of the stopwatch and main loop.
- **button.c**: Functions to handle button press logic.
- **gpio.c**: GPIO configuration and handling.
- **timer.c**: Timer initialization and interrupt handling.
- **segment_map.h**: 7-segment display segment mapping.

## ⚙️ How to Use

1. **Build and Flash**: Compile the code using your preferred IDE (e.g., STM32CubeIDE) and flash the binary to your STM32F407 microcontroller.
2. **Press Start/Stop**: Press the **Start/Stop** button (PC14) to toggle the stopwatch on/off.
3. **Press Reset**: Press the **Reset** button (PC13) to reset the stopwatch to 0.

## 📑 Notes
- The project is built using bare-metal programming techniques, without the use of any RTOS or external libraries.
- The timer interrupt service routine is implemented to handle the timing with high precision, ensuring the stopwatch works correctly with milliseconds and seconds.
- You can modify the timer interval or adjust the behavior of the stopwatch as needed for your application.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Feel free to contact me if you have any questions or issues with the project!
