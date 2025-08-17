[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/NJLWAR4a)
 
![Docs Added](https://github.com/edl-iitb/edl-25-project-submission-edl25_tue16/actions/workflows/classroom.yml/badge.svg)
 
<!-- DON'T MODIFY ANYTHING ABOVE -->
<!-- Modify from here -->
## 📄 Project Documentation

###  Project Name: Universal Wireless Programmer
###  Team Number: TUE-16
###  Team Members:

Anay Malviya 22b1255

Dhruv Meena 22b1279

Hardik Pratap Gohil 22b1293

Kaustav Saha 22b1228

Saptarshi Biswas 22b1258

---

### 📌 Problem Statement and Solution:

Whether it's deep inside big industrial machines, remote terrains or disaster zones, there is always bots and embedded devices to perform critical tasks and send back important data to the users. The big limitation here is that **we have to physically access them to reprogram them and download data.** The **Universal Wireless Programmer** completely cuts off all cords as well as this big limiation. We designed a **wireless programming** and **data retrieval** system for microcontrollers and development boards like Arduino Uno and ATmega328P. We use **RF communication** to send compiled code and receive real-time data back, all over-the-air.

---

### 📚 Contents

- [src](./src) – Source code including firmware and scripts
- [pcb](./pcb) – PCB design files and schematics
- [reports](./reports) – Milestone reports
- [others](./others) – Other circuit diagrams and misc. info
- [3d_models](./3d_models) – 3D CAD files
- [README.md](./README.md) – Problem Statement
- [bom.xls](./bom.xls) – Bill of Materials

---

### 💡 Learnings
We tried a lot of approaches throughout the project to figure out programming since there were no easily available resources related to that. Firstly, we got stuck at figuring out how to make arduino enter into the bootloader mode. For that we tried a bunch of approaches:
1) Software reset - Using DTR and RTS (Failed due to no response from arduino)
2) Power reset - Using pmos (Failed due to insufficient current to turn on the arduino)
3) Power reset - Using power mosfet (Reset worked but failed to enter bootloader mode)
4) Using reset pin - Reset worked but failed to enter bootloader mod

There were some more variations of these methods and some more approaches which all failed until we randomly figured out that the RESET PULSE DURATION which we found in the documentation and used without questioning was not sufficient and tuned it to a higher value which worked

We also spent a lot of time on figuring out USB CDC which was not working. In this task, we were supposed to figure out communication between STM in USB Host mode and arduino as a CDC device. We got no response from arduino whatsoever in this step, so we moved on to using UART for programming.

As soon as we received the RF PCB, we tested it and it was working completely fine. So we started working on implementing the SPI protocol to use the SI4464 PCB for wireless communication. We thought the issue was in software and spent a lot of time figuring out the configuration of the drivers. However, later we checked the hardware again and found a VDD-ground short. We believe that this was due to lack of ESD protection on the IC. We implemented our backup using bluetooth and are working on the PCB 

These were the difficulties we faced but we found out alternative ways to get each of them working to get to a working prototype.

---

*This project was completed as a part of EDL Lab in Spring'25*
