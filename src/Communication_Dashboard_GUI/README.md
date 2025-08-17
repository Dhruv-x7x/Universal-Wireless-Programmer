## Communication Dashboard GUI

This Python-based graphical user interface (GUI) tool provides a comprehensive solution for programming and interacting with embedded devices, specifically STM32 microcontrollers and target devices such as Arduino/ATMEGA328P. Our list of features include:

- **Hex File Upload & Generation**  
  Seamlessly upload hex-files from your system or generate hex files by compiling the Arduino code in the GUI. Ensure that you have ArduinoIDE installed in your local machine if you want to use the GUI to compile code and generate hex-file directly. 

- **Port Detection**  
  Automatically detect and verify the available serial ports for communication with the target device.

- **Signature Verification**  
  Ensure that the device signature on the STM32 matches the target device (Arduino/ATMEGA328P), guaranteeing proper identification.

- **Wireless Transmission**  
  Facilitate reliable wireless transmission of data to the target device.

- **Programming Status**  
  Receive clear feedback on whether the target device has been successfully programmed, with detailed status indicators on the GUI such as progress bars.

- **Serial Monitoring**  
  Monitor serial communication in real-time to track the status of data transfer and device responses.

- **Data Visualization**  
  Display real-time plots and data visualizations directly within the GUI for enhanced monitoring and analysis of the programming process.

---

## Installation

Follow these steps to install and run the application.

### Step 1: Clone the repository
First, clone the repository to your local machine using Git:

```bash
git clone https://github.com/edl-iitb/edl-25-project-submission-edl25_tue16.git
cd edl-25-project-submission-edl25_tue16/src/Communication_Dashboard_GUI
```

### Step 2: Create a Virtual Environment (Optional)

```bash
python -m venv venv
# On Windows, run this next: venv\Scripts\activate
# On macOS/Linux, run this next: source venv/bin/activate
```

### Step 3: Install Dependencies

```bash
pip install -r requirements.txt
```

### Step 4: Run the GUI

```bash
python GUI.py
```

---

## Using the GUI

#### Before you use our GUI ensure the following things:
- You have ArduinoIDE installed on your local machine.
- The transmitting PC should have the UART-HC-05 modules connected to its USB port.
- Install PL2303 Drivers for the HC-05 modules.
- The receiver PC should have the STM32-Arduino/ATMEGA328P modules connected to its USB port. Make the connections between STM32-Arduino/ATMEGA328P as given in `others/uart_schematic.jpeg`.
- A second HC-05 module must be connected to the STM32 and be successfully paired with the HC-05 on the transmitter side. You can find out how to pair HC-05 [here](https://howtomechatronics.com/tutorials/arduino/how-to-configure-pair-two-hc-05-bluetooth-module-master-slave-commands/).
- You have run `./STM32/UniversalWirelessProgrammer/Core/Src/main.c` on the STM32 board without errors. Refer [here](https://github.com/edl-iitb/edl-25-project-submission-edl25_tue16/blob/main/src/STM32/README.md) for the complete instructions on how to run `main.c` on STM32CubeIDE.
- You have familiriazed yourself with STK500 protocol which is used by Arduino and Atmel microcontrollers. Refer [here](https://ww1.microchip.com/downloads/en/AppNotes/doc2591.pdf). 

---

#### Follow these instructions to use our GUI:

- As soon as the GUI window pops up, you will notice there are already some logs in `Activity Log` window. This is supposed to happen as the GUI detects the Arduino CLI for code compilation.

![Step 1](https://drive.google.com/uc?export=view&id=1B0wHLjO6VbWmre7yjgaZWTwCHXHtbLVV)

- Under `Board Selection`, select the target device you are programming. Current implementation only supports ArduinoUNO. Next, click on `Select Board` button and click `OK`.
- If you have already generated a hex-file before, click on `Load HEX File` button and find your hex-file.

![Step 2](https://drive.google.com/uc?export=view&id=1kEtAWT8cMpPXfDxLMJ6y3Phf5U8kKwbo)

- If you want to compile the code on the GUI itself, click the `Arduino Code Editor` menu on the top and then write your code.
- Next, select the correct baudrate from the dropdown list. We have used **38400** in our case.
- Next to `COM Port`, select your port. If no port is visible, please click `Refresh Ports` and try again. Click `Connect`. If this was successful it will show **Connected** in `Activity Log` as well as below the `Refresh Ports` button in green. If you encounter an error, refer here.
- We are ready to program the target device now. Click `Check Signature`, if the target device was ArduinoUNO or ATMEGA328P you should receive back `1e 95 0f`. As well as `STK_INSYNC` and `STK_OK` after it. You should see `Signature Verified` on the top right corner. Error? Refer here.

![Step 4](https://drive.google.com/uc?export=view&id=1e8m-1k5JMW8QTZRmhewlM_MQBtBW653B)

- Once the signature matching is successful, you can program the device whenever you want. Click `Program Device` and wait for the progress bar to finish.
- You can start serial monitoring by clicking `Serial Mode` on the bottom menu and serial plotting by clicking on `Serial Data Plotter` from the top menu bar.
