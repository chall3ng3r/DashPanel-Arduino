# DashPanel-Arduino
Dashboard panel for vehicles using ESP8266, OLED displays, GPS and DS1820 temperature sensors using Arduino framework.

![DashPanel Screens](https://github.com/chall3ng3r/DashPanel-Arduino/blob/master/resources/doc-images/DashPanel-Photo-1.jpg)

THis project uses following components:

* 1x NodeMCU compatible DevKit (ESP8266)
* 2x OLED 0.96" displays with I2C interface
* 2x DS1820 temperature sensors (cabin & ambient)
* 1x GPS module (blox NEO-6M)

Arduino libraries used:

* TinyGPSPlus
* AsyncDelay
* OLED display by Squix
* DallasTemperature

GPIO connections:

* Software Serial for GPS module: 13, 15
* I2C for OLED displays: 4, 5
* OneWire DS1820 Sensors: 2

Photos of final product will be added later. Currently the project is living on breadboard.
![DashPanel on Breadboard](https://github.com/chall3ng3r/DashPanel-Arduino/blob/master/resources/doc-images/DashPanel-Photo-2.jpg)
