# serenity3d
Arduino code controller for LEDs, servos, and audio player running in a custom 3D-printed Serenity (from Firefly)

v0.05 migrates from Arduino UNO to multi-threaded Arduino NANO 33 BLE Sense

Code is segmented into main areas of operation:
- Firefly Drive: controls Adafruit 16-LED Neopixel ring in Serenity's Firefly Drive section
- Grav Ring: controls Adafruit 12-LED Neopixel ring in Serenity's aft gravity ring
- Turbines: controls two WS2812B LED modules in Serenity's nacelle turbine engines
- Cabin: controls three WS2812B LED modules in Serenity's main cabin
- Cockpit: controls a single WS2812B LED module in Serenity's fore cockpit
- PWM: signals the PCA9685 PWM daughterboard to control the two servos for rotating Serenity's nacelles, as well as the two brightLED modules for Serenity's landing lights, and four brightLED mini modules for the port and starboard navigation beacon lights on her nacelles
- Audio: controls the DFPlayerMini daughterboard to play audio MP3/WAV files
- Bluetooth: controls RX/TX for the NANO 33 BLE Sense bluetooth module (migration from HM-10 daughterboard complete)


![Wiring diagram](https://github.com/grandexperiements/serenity3d/photos/blob/master/wiring-diagram.jpg?raw=true)


TODO:
- Convert all segments of operation to distinct C++ classes
- photo documentation
