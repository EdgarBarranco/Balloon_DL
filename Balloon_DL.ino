/*
High altitude baloon data logger - Gloversville High School
By Edgar / KC2UEZ 
12-9-2015 Rev 1

This code reads a series of sensors and stores their values to a SD card. Tested on an Arduino Mega 2560.
CSV can be used http://www.gpsbabel.org/ to gneerate Google earth flight paths.
All the electronics runs from a 3S LiPO 11.1 V Battery
Picture: http://imgur.com/Kb32pvZ

Seonsors: 

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

Extra code:

SD Card
	Uses:
		SD modified librardy from: https://github.com/adafruit/SD
	Connected: D10..D13
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <SPI.h>
#include <SD.h>

// Global variables
struct DATA
{
	unsigned long counter;
	int gps_h, gps_m, gps_s, gps_ms;
	int gps_day, gps_mo, gps_y;
	int gps_sat, gps_fix, gps_fixq;
	float gps_lat, gps_lon, gps_alt, gps_speed, gps_angle;
	float ds_c, ds_f;
	float mpl_bar, mpl_altm, mpl_tc, mpl_tf;
	float si_vis, si_ir, si_uv;
	int lsm_accX, lsm_accY, lsm_accZ;
	int lsm_magX, lsm_magY, lsm_magZ;
	float dht_h, dht_c, dht_f, dht_hi;
	int g_cpm, g_cps;
	float g_uSv_hr;
	char g_rate;
	long int_vcc;
};

DATA sample;

boolean echo = false; 		// Serial debugging output
int debug_rate = 9600;		// Debug baud rate - 2400 for RF / 9600 for Console
float magic_val = 0.0066; 	// 1 for the A/D = 0.0048mV but was changed to match DMM
int min_int_vcc = 3700;		// Minimum value for internal VIN
int ledPin = 13;
int RPI_Pin = 4;			// Pin connected to Raspberry Pi GPIO
boolean RPI_UP = true;		// State of the RPI
void init_file(void);		
boolean check_battery(void);

// DHT22
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// DS18B20
#define ONE_WIRE_BUS 6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// MPL3115A2
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// SI1145
Adafruit_SI1145 uv = Adafruit_SI1145();

// LSM303
Adafruit_LSM303 lsm;

// GPS
Adafruit_GPS GPS(&Serial1);

// Geiger
String readString;
int commaLocations[7];

// SD Card
#define SD_CS 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_CLK 13
char filename[16];

// Sleep mode
int fadeRate = 2;
int maxVal = 255;
int minVal = 0;
int delayVal = 20;

void setup()
{
	if (echo) Serial.begin(debug_rate);
	sample.counter = 0;
	
	if (echo) Serial.println("Start");
	
	// Si1145
	if (! uv.begin())
	{
		if (echo) Serial.println("Si1145. Check your wiring!");
		while (1);
	}
	if (echo) Serial.println("Si1145 Started");

	// LSM303
	if (!lsm.begin())
	{
		if (echo) Serial.println("LSM303. Check your wiring!");
		while (1);
	}
	if (echo) Serial.println("LSM303 Started");

	// MPL3115A2
	if (! baro.begin())
	{
		if (echo) Serial.println("MPL3115A2. Check your wiring!");
		while (1);
	}
	if (echo) Serial.println("MPL3115A2 Started");

	// DS18B20
	DS18B20.begin();
	if (echo) Serial.println("DS18B20 Started");

	// GPS
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);
	delay(1000);
	Serial1.println(PMTK_Q_RELEASE);
	if (echo) Serial.println("GPS Started");

	// Geiger counter
	Serial2.begin(9600);
	Serial2.setTimeout(10000);
	if (echo) Serial.println("Geiger Counter Started");

	// SD
	if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_CLK))
	{
		if (echo) Serial.println("Card init. failed!");
		while(1);
	}
	int n = 0;
	snprintf(filename, sizeof(filename), "0%03d.csv", n);
	while(SD.exists(filename))
	{
		snprintf(filename, sizeof(filename), "0%03d.csv", n++);
	}
	File dataFile = SD.open(filename, FILE_READ);
	if (echo) 
	{ 
		Serial.print("Filename to use: ");
		Serial.println(filename); 
	}
	dataFile.close();
	if (check_battery())
			init_file();
	if (echo) Serial.println("File created!");
	
	if (echo) Serial.println("Setup Finished!");
}

void loop()
{
	if (check_battery())
	{
		if (echo) Serial.println("===================");
		sample.counter++;
		readMPL3115A2();	// MPL3115A2
		readSI1145();		// Si1145
		readLSM303();		// LSM303
		readDS18B20();		// DS18B20
		readDHT22();		// DHT22
		readGPS();			// GPS
		readGeiger();		// Geiger
		readVcc();			// Read internal VCC
		if (check_battery()) 
			writeSD();		// SD
		delay(10000);
	}
	else
		sleepMode();
}

void readMPL3115A2()
{
	sample.mpl_bar = baro.getPressure();
	sample.mpl_bar /= 3377;
	sample.mpl_altm = baro.getAltitude();
	sample.mpl_tc = baro.getTemperature();
	sample.mpl_tf = (sample.mpl_tc * 9.0) / 5.0 + 32;
	if (echo)
	{
		Serial.println("MPL3115A2:");
		Serial.print("   Barometric Presure: ");
		Serial.print(sample.mpl_bar);
		Serial.print(" Inches (Hg) | Altitude: ");
		Serial.print(sample.mpl_altm);
		Serial.print(" m | Temp C ");
		Serial.print(sample.mpl_tc);
		Serial.print(" | Temp F ");
		Serial.println(sample.mpl_tf);
	}
}

void readSI1145()
{
	sample.si_vis = uv.readVisible();
	sample.si_ir = uv.readIR();
	sample.si_uv = uv.readUV();
	sample.si_uv /= 100;

	if(echo)
	{
		Serial.println("SI1145:");
		Serial.print("   Vis: ");
		Serial.print(sample.si_vis);
		Serial.print(" | IR: ");
		Serial.print(sample.si_ir);
		Serial.print(" | UV: ");
		Serial.println(sample.si_uv);
	}
}

void readLSM303()
{
	lsm.read();
	sample.lsm_accX = (int)lsm.accelData.x;
	sample.lsm_accY = (int)lsm.accelData.y;
	sample.lsm_accZ = (int)lsm.accelData.z;
	sample.lsm_magX = (int)lsm.magData.x;
	sample.lsm_magY = (int)lsm.magData.y;
	sample.lsm_magZ = (int)lsm.magData.z;

	if (echo)
	{
		Serial.println("LSM303:");
		Serial.print("   Accel X: ");
		Serial.print(sample.lsm_accX);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(sample.lsm_accY);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.println(sample.lsm_accZ);
		Serial.print("   Compass X: ");
		Serial.print(sample.lsm_magX);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(sample.lsm_magY);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.println(sample.lsm_magZ);
	}
}

void readDS18B20()
{
	DS18B20.requestTemperatures();
	sample.ds_c = DS18B20.getTempCByIndex(0);
	sample.ds_f = (sample.ds_c * 9.0) / 5.0 + 32;

	if (echo)
	{
		Serial.println("DS18B20:");
		Serial.print("   Temp C ");
		Serial.print(sample.ds_c);
		Serial.print(" | Temp F ");
		Serial.println(	sample.ds_f);
	}
}

void readDHT22()
{
	sample.dht_h = dht.readHumidity();
	sample.dht_c = dht.readTemperature();
	sample.dht_f = dht.readTemperature(true);
	sample.dht_hi = dht.computeHeatIndex(sample.dht_f , sample.dht_h );

	if (echo)
	{
		Serial.println("DHT22:");
		Serial.print("   Humidity: ");
		Serial.print(sample.dht_h);
		Serial.print(" | Temp C ");
		Serial.print(sample.dht_c);
		Serial.print(" | Temp F ");
		Serial.print(sample.dht_f);
		Serial.print(" | Heat Index: ");
		Serial.println(sample.dht_hi);
	}
}

void readGPS()
{
	for ( int i = 0; i < 6 ; i++)
	{
		while(!GPS.newNMEAreceived())
		{
			char c = GPS.read();
		}
		GPS.parse(GPS.lastNMEA());
	}

	sample.gps_h = GPS.hour;
	sample.gps_m = GPS.minute;
	sample.gps_s = GPS.seconds;
	sample.gps_ms = GPS.milliseconds;
	sample.gps_day = GPS.day;
	sample.gps_mo = GPS.month;
	sample.gps_y = GPS.year;
	sample.gps_sat = (int)GPS.satellites;
	sample.gps_fix = (int)GPS.fix;
	sample.gps_fixq = (int)GPS.fixquality;
	sample.gps_lat = GPS.latitudeDegrees;
	sample.gps_lon = GPS.longitudeDegrees;
	sample.gps_alt = GPS.altitude;
	sample.gps_angle = GPS.angle;
	sample.gps_speed = GPS.speed;
	sample.gps_speed *=   1.15;

	if (echo)
	{
		Serial.println("GPS:");
		Serial.print("   Time: ");
		Serial.print(sample.gps_h, DEC);
		Serial.print(':');
		Serial.print(sample.gps_m, DEC);
		Serial.print(':');
		Serial.print(sample.gps_s, DEC);
		Serial.print('.');
		Serial.print(sample.gps_ms);
		Serial.print(" | Date: ");
		Serial.print(sample.gps_day, DEC);
		Serial.print('/');
		Serial.print(sample.gps_mo, DEC);
		Serial.print("/20");
		Serial.println(sample.gps_y, DEC);
		Serial.print("   Fix: ");
		Serial.print(sample.gps_fix);
		Serial.print(" | Quality: ");
		Serial.print(sample.gps_fixq);
		Serial.print(" | Satellites: ");
		Serial.println(sample.gps_sat);
		if (sample.gps_fix)
		{
			Serial.print("   Location Deg: ");
			Serial.print(sample.gps_lat, 5);
			Serial.print(", ");
			Serial.print(sample.gps_lon, 5);
			Serial.print(" | Altitude: ");
			Serial.print(sample.gps_alt);
			Serial.print(" | Speed: ");
			Serial.print(sample.gps_speed);
			Serial.print(" MPH | Angle: ");
			Serial.println(sample.gps_angle);
		}
	}
}

void readGeiger()
{
	char readBuffer[64];

	String CPS = "", CPM = "", uSv_hr = "", rate = "";
	for ( int i = 0; i < 3 ; i++)
	{
		Serial2.flush();
		Serial2.readBytesUntil('\n', readBuffer, 64);
	}
	readString = readBuffer;
	if (readBuffer[0] == 'C' && readBuffer[1] == 'P' && readBuffer[2] == 'S')
	{
		if (FindCommaLocations())
		{
			CPS = readString.substring(commaLocations[0] + 1, commaLocations[1]);
			CPM = readString.substring(commaLocations[2] + 1, commaLocations[3]);
			uSv_hr = readString.substring(commaLocations[4] + 1, commaLocations[5]);
			rate = readString.substring(commaLocations[5] + 1, commaLocations[5] + 3);
		}
	}

	sample.g_cps = CPS.toInt();
	sample.g_cpm = CPM.toInt();
	char tmp[uSv_hr.length() + 1];
	uSv_hr.toCharArray(tmp, sizeof(tmp));
	sample.g_uSv_hr = atof(tmp);
	sample.g_rate = rate.charAt(1);

	if (echo)
	{
		Serial.println("Geiger:");
		Serial.print("   CPS: ");
		Serial.print(sample.g_cps);
		Serial.print(" | CPM: ");
		Serial.print(sample.g_cpm);
		Serial.print(" | uSv/hr: ");
		Serial.print(sample.g_uSv_hr);
		Serial.print(" | Rate: ");
		Serial.println(sample.g_rate);
	}
}

bool FindCommaLocations()
{
	commaLocations[0] = readString.indexOf(',');
	commaLocations[1] = readString.indexOf(',', commaLocations[0] + 1);
	commaLocations[2] = readString.indexOf(',', commaLocations[1] + 1);
	commaLocations[3] = readString.indexOf(',', commaLocations[2] + 1);
	commaLocations[4] = readString.indexOf(',', commaLocations[3] + 1);
	commaLocations[5] = readString.indexOf(',', commaLocations[4] + 1);
	commaLocations[6] = readString.indexOf(',', commaLocations[5] + 1);
	if ( commaLocations[6] != 0)
		return true;
	else
		return false;
}

void readVcc() {
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  sample.int_vcc = ADCL;
  sample.int_vcc |= ADCH<<8;
  sample.int_vcc = 1126400L / sample.int_vcc; // Back-calculate AVcc in mV

  if (echo)
  {
	  Serial.println("Internal Voltage: ");
	  Serial.print("   Volts = " );	
	  Serial.println(sample.int_vcc);
  }
}

void writeSD()
{
	File dataFile = SD.open(filename, FILE_WRITE);
	if (dataFile)
	{
		dataFile.print(sample.counter);		dataFile.print(",");
		dataFile.print(sample.gps_y);		dataFile.print("/");
		dataFile.print(sample.gps_mo);		dataFile.print("/");
		dataFile.print(sample.gps_day);		dataFile.print(",");
		dataFile.print(sample.gps_h);		dataFile.print(":");
		dataFile.print(sample.gps_m);		dataFile.print(":");
		dataFile.print(sample.gps_s);		dataFile.print(",");
		dataFile.print(sample.gps_lat,5);	dataFile.print(",");
		dataFile.print(sample.gps_lon,5);	dataFile.print(",");
		dataFile.print(sample.gps_alt,2);	dataFile.print(",");
		dataFile.print(sample.gps_fixq);	dataFile.print(",");
		dataFile.print(sample.gps_sat);		dataFile.print(",");
		dataFile.print(sample.gps_speed,2);	dataFile.print(",");
		dataFile.print(sample.ds_c,2);		dataFile.print(",");
		dataFile.print(sample.ds_f,2);		dataFile.print(",");
		dataFile.print(sample.mpl_bar);		dataFile.print(",");
		dataFile.print(sample.mpl_altm);	dataFile.print(",");
		dataFile.print(sample.mpl_tc);		dataFile.print(",");
		dataFile.print(sample.mpl_tf);		dataFile.print(",");
		dataFile.print(sample.si_vis);		dataFile.print(",");
		dataFile.print(sample.si_ir);		dataFile.print(",");
		dataFile.print(sample.si_uv);		dataFile.print(",");
		dataFile.print(sample.lsm_accX);	dataFile.print(",");
		dataFile.print(sample.lsm_accY);	dataFile.print(",");
		dataFile.print(sample.lsm_accZ);	dataFile.print(",");
		dataFile.print(sample.lsm_magX);	dataFile.print(",");
		dataFile.print(sample.lsm_magY);	dataFile.print(",");
		dataFile.print(sample.lsm_magZ);	dataFile.print(",");
		dataFile.print(sample.dht_h);		dataFile.print(",");
		dataFile.print(sample.dht_c);		dataFile.print(",");
		dataFile.print(sample.dht_f);		dataFile.print(",");
		dataFile.print(sample.dht_hi);		dataFile.print(",");
		dataFile.print(sample.g_cps);		dataFile.print(",");
		dataFile.print(sample.g_cpm);		dataFile.print(",");
		dataFile.print(sample.g_uSv_hr);	dataFile.print(",");
		dataFile.print(sample.g_rate);		dataFile.print(",");
		dataFile.print(sample.int_vcc);		dataFile.print(",");
		dataFile.println();
		dataFile.close();
	}
	else
		if (echo) Serial.println("Error writing data to SD!!");
}

void sleepMode()
{
	for (int fadeValue = minVal ; fadeValue <= maxVal; fadeValue += fadeRate)
	{
		analogWrite(ledPin, fadeValue);
		delay(delayVal);
	}
	delay (1000);
	check_battery();
	for (int fadeValue = maxVal ; fadeValue >= minVal; fadeValue -= fadeRate)
	{
		analogWrite(ledPin, fadeValue);
		delay(delayVal);
	}
	delay (1000);
	
	if (sample.int_vcc > min_int_vcc)
		digitalWrite(ledPin,LOW);
}

boolean check_battery()
{
  readVcc();
  if ( sample.int_vcc > min_int_vcc)
   return true;
  else
    return false;
}

void init_file(void)
{
	File dataFile = SD.open(filename, FILE_WRITE);
	dataFile.print("name,utc_d,utc_t,lat,lon,alt,fix,sat,speed,");	// From GPS
	dataFile.print("temp,tempf,"); 					// From DS18B20
	dataFile.print("MPL_bar,MPL_altm,MPL_temp,MPL_tempf,"); 	// From MPL3115A2
	dataFile.print("SI_vis,SI_ir,SI_uv,");				// From Si1145
	dataFile.print("LSM_accX,LSM_accY,LSM_accZ,");			// From LSM303
	dataFile.print("LSM_magX,LSM_magY,LSM_magZ,");			// From LSM303
	dataFile.print("DHT_h,DHT_temp,DHT_tempf,DHT_hi,");		// From DHT22
	dataFile.print("CPS,CPM,uSv_hr,rate,");				// From Geiger
	dataFile.println("Internal_Vcc");				// From Internal VCC
	dataFile.close();
}
