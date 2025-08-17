## STM32 Files

To run the STM32 programs, you will need to download STM32CubeIDE v1.18 from [here](https://www.st.com/en/development-tools/stm32cubeide.html). Setup the IDE as given on their website and open the `.project` file of the program you want to run on the STM32. If you only want to see the `main.c` file then go to `<directory-of-program-you-want-to-run>/Core/Src/main.c` within the STM32 directory.

First set up the GUI using the instructions given in `src/Communication_Dashboard_GUI/README.md`.

Follow these instructions to Run the programs on NUCLEO STM32-F401RE (you can run it on any NUCLEO STM32-FXXX series as long as it supports USB CDC):
- Download the entire directory you want to run on STM and then click `.project`. Navigate to `Core/Src/main.c`.
- Connect the STM32 to your PC via the cable given with the board.
- Under `Run`, click on `Debug Configurations`
- Go to `Debugger` and check `ST-LINK S/N`. Click `Scan`.
- If nothing is detected, please debug as given [here](https://stackoverflow.com/questions/44703157/no-st-link-detected-error-message-when-trying-to-connect-with-st-link-utility).
- If ST-LINK is detected, click `Apply` and then `Run`. If there are no errors, the board will be programmed successfully.

---

### Directory Structure

```plaintext
├── STM32
  ├── Receiver_Signature_LED_program
  ├── USB_Detect
  ├── HardcodeRXTX
  ├── UniversalWirelessProgrammer
```

#### Directory Descriptions
- Receiver_Signature_LED_program: Working Implementation of transfering hex-files through HC-05
- USB_Detect: Detecting USB devices using USB Enumeration Processes and through a female USB breakout board
- HardcodeRXTX: Complete Implementation of Programming Arduino via UART RX/TX
- UniversalWirelessProgrammer: Final Submission, incorporates the GUI with HardcodeRXTX

*Note: The above directories are solely for the STM32CubeIDE and can not contain README.md files. Therefore their descriptions are given here.*

---

## Using Universal Wireless Progammer with GUI

The state diagram given below explains the core working of the code. The STM is responding to the commands defined by us to perform all actions.

![Flowchart](https://drive.google.com/uc?export=view&id=1t1z4gBN6ZWjRmW8NPOa3Un-jqANbiw6t)



## Using STM

The STM has 3 wires correspondings to RESET(PB4), RX(PA10), TX(PA9) which need to be connected to arduino's or ISP's reset , TX and RX pins. Full connections are given in `others/uart_schematic.jpeg`.
