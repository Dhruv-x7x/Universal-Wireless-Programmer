# Arduino UNO

---

This directory contains the various scripts we used when programming Arduino UNO. To use them, simply download them and open them with ArduinoIDE or copy the code directly into ArduinoIDE. The scripts can be used for other Arduino boards by changing the `pin` definitions at the top. All Arduino code is used as hex-files. To generate hex-files, click `Compile as Binary` under `Sketch` tab of the ArduinoIDE. All code available here are in public domain.

The structure of the directory is as follows:

```plaintext
├── Blink.c
├── Bare_minimum.c
├── Fading.c
├── demo_smart_cooling_system.c
├── Liquid_crystals.c
├── README.md
```

---

- `Blink.c` simply turns LED off and on with a delay of 1 second. The current definition is for `Digital Pin 10` maps to `Pin 16` of the ATMEGA328P IC used on Arduino UNO.
- `Bare_minimum.c` is basically an empty file that is used for debugging purposes only.
- `Fading.c` makes the LED at pin 10 fade in and out.
- `ArduinoISP.ino` is another example code that we used when testing ISP protocol. We programmed the ATMEGA328P IC using Arduino UNO as a ISP programmer. To open this file in your ArduinoIDE go to `Examples > 11. ArduinoISP > ArduinoISP`. Please refer to the official documentation for the connections and further explanation. [Here](https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP/)
- `demo_smart_cooling_system.c` is code for our demonstration. To use this code, please refer to the connections given [here](https://github.com/edl-iitb/edl-25-project-submission-edl25_tue16/blob/main/others/demo_smart_cooling_system_schematic.jpeg). The goal of this demo is to have a fan that cools a temperature sensor **more** if the temperature rises by rotating faster and slowing down when the temperature drops (feedback loop).
