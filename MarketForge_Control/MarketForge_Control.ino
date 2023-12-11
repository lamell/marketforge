#include <Arduino.h>
#include <Adafruit_MAX31865.h> 
#include <vfd_nec_fc20x2.h>

Adafruit_MAX31865 tempSensor = Adafruit_MAX31865(PC5, PC7, PC6, PA1);


VFD_NEC_FC20X2 vfd;

#define   RREF      430.0
#define   RNOMINAL  100.0

void setup() {
  // put your setup code here, to run once:

  bool res = tempSensor.begin(MAX31865_4WIRE);
  Serial3.begin(9600);
  vfd.setSerialPort(&Serial3);
  vfd.Init();

}

void loop() {
  char txt[50];
  sprintf(txt,"150");
  // put your main code here, to run repeatedly:
  vfd.Print(txt,0);
  
}

