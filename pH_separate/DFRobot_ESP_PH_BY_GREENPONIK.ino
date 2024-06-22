/*
 * file DFRobot_ESP_PH_BY_GREENPONIK.ino
 * @ https://github.com/GreenPonik/DFRobot_ESP_PH_BY_GREENPONIK
 *
 * This is the sample code for Gravity: Analog pH Sensor / Meter Kit V2, SKU:SEN0161-V2
 * In order to guarantee precision, a temperature sensor such as DS18B20 is needed, to execute automatic temperature compensation.
 * You can send commands in the serial monitor to execute the calibration.
 * Serial Commands:
 *   enterph -> enter the calibration mode
 *   calph   -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
 *   exitph  -> save the calibrated parameters and exit from calibration mode
 * 
 * Based on the @ https://github.com/DFRobot/DFRobot_PH
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * ##################################################
 * ##################################################
 * ########## Fork on github by GreenPonik ##########
 * ############# ONLY ESP COMPATIBLE ################
 * ##################################################
 * ##################################################
 * 
 * version  V1.1
 * date  2019-06
 */

#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include <Adafruit_ADS1X15.h>


#define SDA_PIN 21
#define SCL_PIN 22
#define ADS1115_ADDR_1 0x48 

DFRobot_ESP_PH ph;
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 5000 //the esp voltage supply value
// #define PH_PIN 35		//the esp gpio data pin number
float voltage, phValue, temperature = 25;
float volt_pH;
Adafruit_ADS1115 ads1;


void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
	Serial.begin(115200);
  ads1.begin(ADS1115_ADDR_1);
	EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
	ph.begin();
}

void loop()
{
	static unsigned long timepoint = millis();
	if (millis() - timepoint > 1000U) //time interval: 1s
	{
		timepoint = millis();
		//voltage = rawPinValue / esp32ADC * esp32Vin
    volt_pH = readVoltage(&ads1, 0);
		voltage = volt_pH / ESPADC * ESPVOLTAGE; // read the voltage
		Serial.print("voltage:");
		Serial.println(voltage, 4);
		
		//temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
		Serial.print("temperature:");
		Serial.print(temperature, 1);
		Serial.println("^C");

		phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
		Serial.print("pH:");
		Serial.println(phValue, 4);
	}
	ph.calibration(voltage, temperature); // calibration process by Serail CMD
}

float readTemperature()
{
	//add your code here to get the temperature from your temperature sensor
}
float readVoltage(Adafruit_ADS1115 *ads, int channel)
{
    // Read the ADC value directly from the specified channel
    int16_t adcValue = ads->readADC_SingleEnded(channel);
    Serial.println(adcValue);

    // Convert ADC value to voltage (0-3.3V)
    float voltage = adcValue * (5000.0 / 32767.0);

    return voltage;
}