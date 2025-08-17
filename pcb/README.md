### PCB

---

We only designed one PCB which houses the **SI4464** module to receive and transmit signals. You can find its description [here](https://github.com/edl-iitb/edl-25-project-submission-edl25_tue16/blob/main/pcb/PCB1/README.md). 

All other circuitry was tested on breadboard or soldered on to general purpose boards. Their relevant circuitry is organized under `others` directory.

---

#### Reasons for Issues with PCB

- Vdd and Gnd were shorted as SI4464 IC which didn't have ESD protection was somehow damaged.
   
