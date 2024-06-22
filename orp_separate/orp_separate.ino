/*
# This sample codes is for testing the ORP meter V1.0.
 # Editor : YouYou
 # Date   : 2013.11.26
 # Product: ORP meter
 # SKU    : SEN0165
*/
#include <Adafruit_ADS1X15.h>

#define VOLTAGE 3.30    //system voltage
#define OFFSET 0        //zero drift voltage
#define LED 13         //operating instructions

#define SDA_PIN 21
#define SCL_PIN 22
#define ADS1115_ADDR_1 0x48 
double orpValue;
float volt_orp;

#define ArrayLenth  40    //times of collection
#define orpPin 1          //orp meter output,connect to Arduino controller ADC pin
Adafruit_ADS1115 ads1;

int orpArray[ArrayLenth];
int orpArrayIndex=0;

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    printf("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}


void setup(void) {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(9600);
  ads1.begin(ADS1115_ADDR_1);
  pinMode(LED,OUTPUT);
}

void loop(void) {
  static unsigned long orpTimer=millis();   //analog sampling interval
  static unsigned long printTime=millis();
  if(millis() >= orpTimer)
  {
    orpTimer=millis()+20;
    volt_orp = readVoltage(&ads1, 0);
    orpArray[orpArrayIndex++]=volt_orp;    //read an analog value every 20ms
    if (orpArrayIndex==ArrayLenth) {
      orpArrayIndex=0;
    }
    orpValue=((30*(double)VOLTAGE*1000)-(75*avergearray(orpArray, ArrayLenth)*VOLTAGE*1000/4096))/75-OFFSET;

    //convert the analog value to orp according the circuit
  }
  if(millis() >= printTime)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    printTime=millis()+800;
    Serial.print("ORP: ");
    Serial.print((int)orpValue);
        Serial.println("mV");
        digitalWrite(LED,1-digitalRead(LED));
  }
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