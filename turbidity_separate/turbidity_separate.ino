#include <Adafruit_ADS1X15.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define ADS1115_ADDR_1 0x48 
float volt_turbidity;

Adafruit_ADS1115 ads1;

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200); //Baud rate: 9600
  ads1.begin(ADS1115_ADDR_1);
}
void loop() {
  volt_turbidity = readVoltage(&ads1, 0);
  int sensorValue = volt_turbidity;// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 4096.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(voltage); // print out the value you read:
  delay(500);
}
float readVoltage(Adafruit_ADS1115 *ads, int channel)
{
    // Read the ADC value directly from the specified channel
    int16_t adcValue = ads->readADC_SingleEnded(channel);
    //Serial.println(adcValue);

    // Convert ADC value to voltage (0-3.3V)
    float voltage = adcValue * (5000.0 / 32767.0);

    return voltage;
}

