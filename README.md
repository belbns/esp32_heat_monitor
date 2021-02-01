Scanning and controlling two BLE devices (based on STM32F030F4P6 + YDY-16),<br>
showing main parameters via TFT display (ILI9341),<br>
showing and setting parameters via web server<br>
with using filesystem on sd card for web server files and for saving parameters and statistics.<br>
<br>
Hardware: ESP32-WROOM-32D, ILI9341 based TFT LCD 2.4inch with card reader.<br>
Tools: Espressif ESP-IDF v4.3-dev-2136-gb0150615d, Sublim Text, Linux Mint.<br>
Based mostly on Espressif examples.<br>
Thanks nopnop2002 for ili9340 library (https://github.com/nopnop2002/esp-idf-ili9340).<br>
<br>
It works, but I see it at the moment as a combain test, <br>
that contain web server, BLE gatt client, file system on SD-CARD and time sync.<br>


