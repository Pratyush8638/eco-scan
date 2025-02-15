#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

#define DO_PIN A1

#define VREF 5000    //VREF (mv)
#define ADC_RES 4096 //ADC Resolution


#define SDA_PIN 21
#define SCL_PIN 22
#define ADS1115_ADDR_1 0x48 // Address of the first ADS1115

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0
Adafruit_ADS1115 ads1;

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
float volt_do;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  ads1.begin(ADS1115_ADDR_1);
}

void loop()
{
  Temperaturet = (uint8_t)READ_TEMP;
  volt_do = readVoltage(&ads1, 0);
  // ADC_Raw = analogRead(DO_PIN);
  // ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  //Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  Serial.print("ADC Voltage:\t" + String(volt_do) + "\t");
  Serial.println("DO:\t" + String(readDO(volt_do, Temperaturet)) + "\t");

  delay(1000);
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