# graupner_sumd_to_usb_joystick
Arduino converter from Graupner SUMD protocol to USB Joystick.

My motivation was to be able to use my Graupner MX-20 as a wireless trainer to my RealFlight 8 simulator.

You will need an Arduino with USB-features to build this project. I used an Arduino Leonardo.

I used a Graupner GR12L-SUMD receiver which can output 12 channels on the SUMD protocol. I can then use the momentary switch SW9 as the reset-button.

prerequisites: ArduinoJoystickLibrary library by Matthew Heironimus (https://github.com/MHeironimus/ArduinoJoystickLibrary)

Credit for SUMD example: Holger Lambertus in post: https://www.rc-network.de/threads/wie-kann-man-mit-dem-arduino-nano-das-sumd-signal-lesen.647868/post-6775946



