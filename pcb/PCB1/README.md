## PCB for SI4464 Transceiver 

The **SI4464 PCB** is a **2** layered PCB we designed that acts like a breakout board for the [SI4464 IC](https://www.silabs.com/wireless/proprietary/ezradiopro-sub-ghz-ics/device.si4464?tab=specs). It can be subdivided into 2 major portions:

- **Matching Network**
- **Connections to Header Pins**

---

### Matching Network: 
 - It has a trace (of higher width) coming from the SMA connector (that is attached to the antenna), that goes into a series of inductors and capacitors (the matching network), and finally into the IC. The layout for the matching network was optimised, and was kept similar to the layout given in [layout guidelines](https://www.silabs.com/documents/public/application-notes/AN629.pdf) for the IC.
 - **Decoupling capacitors** were also placed very close to the IC power input ports, as per the layout guidelines.
 - All the traces in these portions were made on the **front copper layer**.

### Header Pins for External Connection:
- We attached this PCB on the STM32 Nucleo-Board which blocked its pins and hence made a separate set of header pins on the top that were shorted to the original Nucleo board pins so that the pins underneath could be accessed from these.
- 2 traces for the connection of the header pin to the IC were done in the **bottom copper layer**.

### Directory Structure

```plaintext
  ├── /PCB1 
    ├── design.kicad_pcb # PCB Design in KiCAD format 
    ├── schematic.sch # KiCAD Schematic
    ├── README.md
    ├── pictures
      ├── ALL_LAYERS_PCB.jpg # Schematic from front
      ├── pcb_board.jpg # PCB Board without antenna
      ├── pcb_with_antenna.jpg # PCB Board with antenna
      ├── pcb_3d_front.jpeg # 3D view from front
      ├── pcb_3d_back # 3D view from back
      ├── pcb_sch_back.jpeg  # Schematic from back
```
