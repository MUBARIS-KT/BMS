#Battery Management System
This project provides code for a Battery Management System (BMS) using Texas Instruments' BQ76940 module with STM32 microcontrollers. Implemented in the STM32 HAL library, this BMS is designed to monitor and manage essential battery parameters, providing reliable protections and ensuring efficient battery operation.

Features
Battery Cell Monitoring: Tracks critical metrics such as cell voltage, current, temperature, State of Charge (SoC), and State of Health (SoH).
Protection Mechanisms: Offers multiple safety protections, including:
Undervoltage Protection
Overvoltage Protection
Overheating Protection
Overcharge Protection
STM32 HAL Integration: Utilizes STM32 HAL functions within STM32CubeIDE for seamless microcontroller interfacing and efficient battery parameter management.
Getting Started
This project is built using STM32CubeIDE, with code written in C and leveraging HAL libraries. It is compatible with any STM32 development board supporting the BQ76940 module and STM32 HAL functions.

Prerequisites
STM32CubeIDE: Required IDE for code development and testing.
Usage
The code is structured to integrate easily into battery management applications, enabling customization of monitoring and protection features based on specific project requirements and hardware configurations.

Technologies Used
Programming Language: C
Libraries: STM32 HAL, STM32CubeIDE for embedded development
