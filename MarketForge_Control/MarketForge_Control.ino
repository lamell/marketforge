#include <Arduino.h>
#include <Adafruit_MAX31865.h> 
#include <vfd_nec_fc20x2.h>

#include "MovingAverage.h"

//MAX31865 Temp Sensor in Chamber
#define MAX31865_CS   PIN_PC5
#define MAX31865_DI   PIN_PC7
#define MAX31865_DO   PIN_PC6
#define MAX31865_CLK  PIN_PA1
Adafruit_MAX31865 tempSensor = Adafruit_MAX31865(MAX31865_CS, MAX31865_DI, MAX31865_DO, MAX31865_CLK);
MovingAverage<float>  avgTemp(9,0.0);

//VFD Display NEC FC20X2
VFD_NEC_FC20X2 vfd;
//Analog Voltage for Pressure Sensor. 30 PSI. 0 PSI = 0.5V, 15 PSI = 2.5V, 30 PSI = 4.5V
//Conversion Formula, without correction: y = 7.5x - 3.75
#define ADC_STEPS   1024
#define AREF_VOLTS  5.0
#define ANALAOG_RES (AREF_VOLTS / (float) ADC_STEPS)
int analogVoltPin = PIN_PF0;
int analogValueInt = 0;
float Pressure = 0.0;
MovingAverage<float>  avgVolts(29,0.0);

#define   RREF      430.0
#define   RNOMINAL  100.0

//Keys for Start and Stop
#define BTN_START PIN_PA3
#define BTN_STOP  PIN_PA5

#define LED_OC  PIN_PL7

void setup() {
  // put your setup code here, to run once:

  bool res = tempSensor.begin(MAX31865_4WIRE);
  Serial3.begin(9600);
  Serial3.print(("Test"));
  vfd.setSerialPort(&Serial3);
  vfd.Init();

  pinMode(BTN_START, INPUT);
  pinMode(BTN_STOP, INPUT);

  //LED
  pinMode(LED_OC, OUTPUT);
}

void loop() {
  char txt[50];
  //Test Analog Voltage
  float voltage = analogRead(analogVoltPin) * ANALAOG_RES;
  avgVolts.Insert(voltage); 
  Pressure = (7.5 * avgVolts.GetAverage()) - 3.9; 
  Pressure = (1.044227 * Pressure) +  0.061628;

  ////Read MAX31865
  uint16_t getMAX31865 = tempSensor.readRTD();
  float    ratio = getMAX31865;
  ratio /= 32768;
  float   Resistance = 0.0;
  Resistance = RREF * ratio;
  avgTemp.Insert(tempSensor.temperature(RNOMINAL, RREF));
  
  char fValue[10];
  dtostrf(avgVolts.GetAverage(), 5, 3, fValue);
  sprintf(txt,"V:%s", fValue);
  dtostrf(Pressure, 5,2, fValue);
  sprintf(txt, "%s, P:%s  ", txt, fValue);
  vfd.Print(txt,0);

  //sprintf(txt,"Resist: %1.2f, Temp: %1.2f  ", Resistance, /* tempSensor.temperature(RNOMINAL, RREF) */ 2.12 );
  dtostrf(Resistance, 5, 2, fValue);
  sprintf(txt,"R:%s", fValue);
  dtostrf(avgTemp.GetAverage(), 5,2, fValue);
  sprintf(txt, "%s, T:%s  ", txt, fValue);
  vfd.Print(txt, 20);

  digitalWrite(LED_OC, !digitalRead(LED_OC));
  delay(100);
  //digitalWrite(LED_OC, 1);
  //delay(500);

}

