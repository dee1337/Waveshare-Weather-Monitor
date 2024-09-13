# ESP32 and Waveshare 4.2 e-ink 3 colour display weather monitor

Changes in this fork:

- Hardware - different setup: 4.2" B/W waveshare module with ESP32-Wroom32-D
- UI Changes:
    - red colors converted to black colors
    - focus on watersports
        - river statistics added in UI (temperature, level and flow) / added new endpoint for that
        - wind speed forecasts added
        - changed wind metrics from mph to km/h
    - deactivated Temp & Feel graph

-----
A revised version (fixed minor errors) of the Weather Display written by G6EJD,
and updated graphics to v2 of the 3 colour Waveshare e-ink display I am using.

This is still in development, need to;
* put the ESP32 to deep sleep
* find a battery/charger solution (e.g. different ESP32 board)
* have a 3D case made

This project was written using Visual Studio Code and the PlatformIO extension.

Libraries used: GxEPD2, ArduinoJson.

Version 1 display.
![alt text width="500"](/pictures/v1.jpg)

Version 2 display.
![alt text width="500"](/pictures/v2.jpg)

Version 3 display.
![alt text width="500"](/pictures/v3.jpg)
