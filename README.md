# Balloon_DL
High Altitude Balloon Data Logger using the Arduino Mega

This project will be use to log relevant data from a high altitude balloon for later study. 

It is built around the Arduino Mega and it collects data from various sensors described below: 

Waterproof DS18B20 Digital temperature sensor
DS18B20 - https://www.adafruit.com/product/381
	Uses:
		OneWire.h - http://www.pjrc.com/teensy/td_libs_OneWire.html
		DallasTemperature.h - http://www.milesburton.com/?title=Dallas_Temperature_Control_Library
	Connected: D6
	
DHT22 temperature-humidity sensor + extras
DHT22 - https://www.adafruit.com/product/385
	Uses: 
		DHT - https://github.com/adafruit/DHT-sensor-library
	Connected: D2	

MPL3115A2 - I2C Barometric Pressure/Altitude/Temperature Sensor	
MPL3115A2 - https://www.adafruit.com/products/1893
	Uses: 
		Adafruit_MPL3115A2.h - https://github.com/adafruit/Adafruit_MPL3115A2_Library
	Connected: I2C 

SI1145 Digital UV Index / IR / Visible Light Sensor
SI1145 - https://www.adafruit.com/product/1777
	Uses: 
		Adafruit_SI1145.h -
		https://forums.adafruit.com/viewtopic.php?f=22&t=59664&p=303930&hilit=si1145+address#p303207
		Based on: https://github.com/adafruit/Adafruit_SI1145_Library
	Connected: I2C

Adafruit Ultimate GPS Logger Shield - Includes GPS Module		
GPS - https://www.adafruit.com/product/1272
	Uses:
		Adafruit_GPS.h - https://github.com/adafruit/Adafruit-GPS-Library
	Connected: Serial1
	
Geiger Counter Kit - Radiation Sensor
Geiger - https://www.adafruit.com/products/483
	Uses: 
		Code based from: http://mightyohm.com/forum/viewtopic.php?t=3431&f=15#p5730
	Connected: Serial2

SD Card
	Uses:
		SD modified librardy from: https://github.com/adafruit/SD
	Connected: D10..D13
