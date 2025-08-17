### Overview

`reports` directory contains all reports and presentations submitted for all milestones. Below is an overview of our project.

---

### Objectives
- **Python-based Communication Dashboard (GUI)**
  - Develop an intuitive graphical interface to upload and parse `.hex` files, initiate wireless connection, and receive as well as plot real-time serial data.
- **Bluetooth Communication using HC-05 Module**
  - Implement a basic communication protocol using the **HC-05** module to wirelessly transmit .hex files from the GUI to the target system in the first prototype.
- **STM32F401RE Board as a Wireless Programmer**
  - Program the STM32 board to receive the transmitted `.hex` file via UART from the HC-05 and program external devices through two different protocols:
    - **USB CDC**: Detect connected devices and program development boards such as the Arduino Uno using **STK500** protocol.
    - **In-System Programming (ISP)**: Implement low-level SPI communication to program standalone microcontrollers like the ATmega328P.
  - Include port detection and signature matching to ensure accurate device targeting and communication.
- **Design a Custom PCB with the SI4464 RF Transceiver**
  - Replace Bluetooth with the **SI4464** RF module in final prototype.
- **Implement Bi-Directional Communication**
  - Add data reception from the programmed device, enabling real-time serial monitoring and plotting on the GUI communication dashboard.

---

### Design Process

---

### Results

We successfully achieved the core objectives of our project by fully implementing the wireless transmission and reception pipeline. Our system enables `.hex` files to be sent over-the-air from our custom-built Python GUI to the STM32F401RE board, which then programs the target device. Once programmed, the target device transmits real-time data back to the STM32, which is displayed via the serial monitor integrated into the GUI.

We also designed and fabricated a custom PCB to replace the Bluetooth module with the SI4464 RF transceiver. However, due to our PCB not working as expected, we decided to go with our backup plan and use Bluetooth for the complete communication pipeline in the final prototype.

We also faced issues with the planned programming methods (USB CDC and ISP), so we implemented a third method using UART RX/TX communication directly with the target device. Due to challenges faced in receiving valid responses from the target using low-level SPI commands in the ISP protocol, and partial implementation of the USB CDC interface, we were forced to use the RX/TX pins. We successfully implemented the STK500 protocol over UART to communicate with the target microcontroller.

To demonstrate the functionality of our project, we wirelessly transmitted and programmed a simple `blink.ino` hex-file. The LED on the target device blinked at different intervals based on the uploaded program, validating our over-the-air programming. We also received simple UART messages on the serial monitor.

---

### Conclusions

Our system presents a practical and effective solution to the problem of remotely programming and communicating with embedded devices. We developed key technologies and wrote custom-firmware for various microcontrollers. There was a lot of learning involved, both technical and personal. We gained hands-on experience with serial communication protocols such as UART, SPI, and USB CDC. The entire protocol along with all the commands are ingrained in our brains. There were many challenges along the way, like in low-level SPI programming and USB CDC setups which we solved after long nights of debugging and research. We learned the importance of backup plans and the hurdles of integrating outdated or vaguely documented protocols. We developed debugging skills at both the hardware and software levels. 

The future scope of this project would include the following:
- **Upgrading from bluetooth to RFIC**: RFIC provides a much longer range of communication.
- **Implement USB/ISP protocols successfully**: Helps to extend to multiple other development boards and controllers.
