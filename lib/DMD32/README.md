# Fast DMD32 
A library for driving the Freetronics 512 pixel dot matrix LED display "DMD", a 32 x 16 layout using ESP32.

Patched to work with  Arduino-ESP32 2.0.3 and later. 

This library is a fork of the [(DMD32)](https://github.com/Qudor-Engineer/DMD32) what was fork of original one [(DMD)](https://github.com/freetronics/DMD) modified to support ESP32 (currently it is only working on ESP32).
This version have optimized SPI transfer to achieve more than 2x times the speed at same clock. Allowing to increase refresh rate with multiple display modules. 

The connection between ESP32 and DMD display shown in the image below:






![](https://github.com/MiyukiDark/DMD32/blob/main/connection.png)


Up to 23 display modules in series can be used with 4MHz SPI clock and refresh timing as in examples resulting in 1.1kHz. SPI clock can be further increased. 


Miyuki
