/*
*
* Comp Board Used:
*   - Kelly Instruments Horiz24 Bd: BLDC_6_Step_Hall_Sensor_IRAMY20UP60B_2_Layer_v3_Mfct_210723
*
* VFD Module used:
*   - NEC FC20X2J
*
* VFD 
* Module Connector     Comp Connector     Signal Name
* P2-1                  +5V               +5V Dig
* P2-2                  GND               Gnd Dig
* P3-33                 PL1-7             Comp TX on Serial Port 3
*
* MAX31865
* 1                     PL2-2             +5V
* 2                     PL2-1             GND
* 3                     N.C.              +3.3V, not used unless system is 3.3V
* 4                     PL2-8             CLCK
* 5                     PL2-5             SDO
* 6                     PL2-6             SDI
* 7                     PL2-4             CS
* 8                     N.C.              DRDY, when donde converting signals this pin. Not needed.
*
* Pressure Sensor. eBay 30PSI. Needs +5V, analog volt out. 0 PSI -> 0.5V, 30 PSI -> 4.5V
* RED                   +5V               Hardwired on board
* BLACK                 GND                 "         "
* GREEN                 U4-1 (R23-R15)    SIG - Analog voltage out
*
*/

#include <Arduino.h>
#include <Adafruit_MAX31865.h> 
#include <vfd_nec_fc20x2.h>

#include "MovingAverage.h"

#include <EEPROM.h>
#include<Ticker.h>
#include <AutoPID.h>
#include <LibPrintf.h>


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
double Temperature, Pressure;

MovingAverage<float>  avgVolts(29,0.0);

/*
  Variable declarations
*/
//EEPOM Variables
int   ee_position;
double EE_Setpoint;
double EE_Proportional;
double EE_Integral;
double EE_Derivative;
float EE_Temp_Cor_Factor;
float EE_Pres_Cor_Fctr;
int EE_Min;
bool EE_Open_Valve_At_End_Of_Cycle;

float LastTemp;
bool InProgram;
bool InCycle;
double Setpoint;
double pidOutput;
double Kp, Ki, Kd;
float Factor_Corr_Temp;
float Factor_Corr_Press;
float Pressure_Offset;
int MinutesSetpoint;

char Disp[21]; //For VFD text

uint16_t ticksSeconds;
uint16_t Seconds, Minutes, Hours;


#define   RREF      430.0
#define   RNOMINAL  100.0

//Keys for Start and Stop
#define BTN_START PIN_PA3
#define BTN_STOP  PIN_PA5

//Vent Solenoid
#define VENT_SOLENOID PIN_PA2 //White
#define VALVE_OPEN    1
#define VALVE_CLOSE   0

//Heater 
#define HEATER1       PIN_PA4 //Green
#define HEATER_ENABLE PIN_PA6
#define HEATER_ON     1
#define HEATER_OFF    0


//LEDs
#define LED0  PIN_PL0
#define LED1  PIN_PL1
#define LED2  PIN_PL2
#define LED3  PIN_PL3
#define LED4  PIN_PL4
#define LED5  PIN_PL5
#define LED6  PIN_PL6
#define LED7  PIN_PL7
#define LED_ON  1
#define LED_OFF 0
#define LED_OC  PIN_PL7
#define HEATER_LED  LED0
#define VALVE_LED   LED7

//PID
#define PID_OUT_LIM_LOW   0
#define PID_OUT_LIM_HIGH  100
AutoPID   pid(&Temperature, &Setpoint, &pidOutput, PID_OUT_LIM_LOW, PID_OUT_LIM_HIGH, Kp, Ki, Kd);

/*
  Structures
*/
enum Autoclave_State {
  ready=0,
  heating,
  sterilizing,
  cooling,
};

uint8_t state;

/*
  Functions
*/
void Read_EEPROM(void) {
  ee_position = 0;
  EEPROM.get(ee_position, EE_Setpoint);
  Setpoint = EE_Setpoint = 121.0;
  ee_position += sizeof(EE_Setpoint);

  EEPROM.get(ee_position, EE_Proportional);
  Kp = EE_Proportional = 20.0;
  ee_position += sizeof(EE_Proportional);

  EEPROM.get(ee_position, EE_Integral);
  Ki = EE_Integral = 0.08;
  ee_position += sizeof(EE_Integral);

  EEPROM.get(ee_position, EE_Derivative);
  Kd = EE_Derivative = 0.0;
  ee_position += sizeof(EE_Derivative);

  EEPROM.get(ee_position, EE_Temp_Cor_Factor);
  Factor_Corr_Temp = EE_Temp_Cor_Factor = 1.043;
  ee_position += sizeof(EE_Temp_Cor_Factor);

  EEPROM.get(ee_position, EE_Pres_Cor_Fctr);
  Factor_Corr_Press = EE_Pres_Cor_Fctr = 0.993;
  ee_position += sizeof(EE_Pres_Cor_Fctr);

  EEPROM.get(ee_position, EE_Min);
  MinutesSetpoint = EE_Min = 15;
  ee_position += sizeof(EE_Min);

  EEPROM.get(ee_position, EE_Open_Valve_At_End_Of_Cycle);

  pid.setGains(Kp,Ki,Kd);
}

void Write_EEPROM(void) {
  ee_position = 0;
  EEPROM.update(ee_position, EE_Setpoint);
  ee_position += sizeof(EE_Setpoint);

  EEPROM.update(ee_position, EE_Proportional);
  ee_position += sizeof(EE_Proportional);

  EEPROM.update(ee_position, EE_Integral);
  ee_position += sizeof(EE_Integral);

  EEPROM.update(ee_position, EE_Derivative);
  ee_position += sizeof(EE_Derivative);

  EEPROM.update(ee_position, EE_Temp_Cor_Factor);
  ee_position += sizeof(EE_Temp_Cor_Factor);

  EEPROM.update(ee_position, EE_Pres_Cor_Fctr);
  ee_position += sizeof(EE_Pres_Cor_Fctr);

  EEPROM.update(ee_position, EE_Min);
  ee_position += sizeof(EE_Min);

  EEPROM.update(ee_position, EE_Open_Valve_At_End_Of_Cycle);
}

void EEPROMDefaults() {
  ee_position = 0;
  EEPROM.update(ee_position, 121.0);
  ee_position += sizeof(EE_Setpoint);

  EEPROM.update(ee_position, 1.0);
  ee_position += sizeof(EE_Proportional);

  EEPROM.update(ee_position, 0.0);
  ee_position += sizeof(EE_Integral);

  EEPROM.update(ee_position, 0.0);
  ee_position += sizeof(EE_Derivative);

  EEPROM.update(ee_position, 1.0);
  ee_position += sizeof(EE_Temp_Cor_Factor);

  EEPROM.update(ee_position, 1.0);
  ee_position += sizeof(EE_Pres_Cor_Fctr);

  EEPROM.update(ee_position, 15);
  ee_position += sizeof(EE_Min);

  EEPROM.update(ee_position, VALVE_OPEN);

}

void Read_Pressure(void) {
  //Test Analog Voltage
  float voltage = analogRead(analogVoltPin) * ANALAOG_RES;
  avgVolts.Insert(voltage); 
  Pressure = (7.5 * avgVolts.GetAverage()) - 3.9; 
  Pressure = (1.044227 * Pressure) + 0.061628;
  Pressure = Factor_Corr_Press * Pressure;
}

void Read_Temperature() {
  ////Read MAX31865
  uint16_t getMAX31865 = tempSensor.readRTD();
  // float    ratio = getMAX31865;
  // ratio /= 32768;
  // float   Resistance = 0.0;
  // Resistance = RREF * ratio;
  avgTemp.Insert(tempSensor.temperature(RNOMINAL, RREF));
  Temperature = avgTemp.GetAverage();

  // Temperature = (1.0528 * Temperature) + 0.7994;
  Temperature = (-0.0003099*pow(Temperature,2)) + (1.1137543*Temperature) - 1.2987112;
  Temperature *= Factor_Corr_Temp;

}

void Standby(void) {
  char txt[21];
  
  sprintf(txt,"Listo              ");
  vfd.Print(txt,0);

  sprintf(txt,"%1.1f C  ", Temperature);
  vfd.Print(txt,20);
  // Pos_VFD(20);
  // Serial1.print(txt);

  if (Pressure >= -0.4 && Pressure <= 0.4) {
    sprintf(txt," %1.1f Psi", 0.0);
    vfd.Print(txt,40-strlen(txt)+0);
  } else {
    sprintf(txt," %1.1f Psi", Pressure);
    vfd.Print(txt,40-strlen(txt)+0);
  }

  digitalWrite(VENT_SOLENOID,VALVE_OPEN);
// PrintRTDStatus(rtd.getStatus());
}

void Heating(void) {
  char txt[20];
  
  sprintf(txt,"Calentando  ");
  vfd.Print(txt,0);

  sprintf(txt,"%02d:%02d:%02d",Seconds/3600, Seconds/60, Seconds % 60);
  vfd.Print(txt,12);

  sprintf(txt,"%1.1f C   ", Temperature);
  vfd.Print(txt,20);

  sprintf(txt," %1.1f Psi ", Pressure);
  vfd.Print(txt,40-strlen(txt));

  digitalWrite(HEATER_ENABLE, HEATER_ON);

  if (Temperature >=97.0) {
    digitalWrite(VENT_SOLENOID,VALVE_CLOSE);
  }
}

void Sterilizing(void) {
  char txt[20];
  
  sprintf(txt,"Esteriliz.  ");
  vfd.Print(txt,0);

  sprintf(txt,"%02d:%02d:%02d",Seconds/3600, Seconds/60, Seconds % 60);
  vfd.Print(txt,12);

  if ((Temperature >= (Setpoint - 0.5))  && (Temperature <= (Setpoint + 0.5))) {
    sprintf(txt,"%1.1f C  ", Setpoint);
    vfd.Print(txt,20);

    sprintf(txt," %1.1f Psi ", 15.0);
    vfd.Print(txt,40-strlen(txt));
  } else {
    sprintf(txt,"%1.1f C   ", Temperature);
    vfd.Print(txt,20);

    sprintf(txt," %1.1f Psi ", Pressure);
    vfd.Print(txt,40-strlen(txt));
  }

}

void Cooling(void) {
  char txt[20];
  
  sprintf(txt,"Enfriando  ");
  vfd.Print(txt,0);

  InCycle = false;

  sprintf(txt,"%02d:%02d:%02d",Seconds/3600, Seconds/60, Seconds % 60);
  vfd.Print(txt,12);

  sprintf(txt,"%1.1f C  ", Temperature);
  vfd.Print(txt,20);

  if (Pressure >=15.0) {
    sprintf(txt," %1.1f Psi ", 15.0);
  vfd.Print(txt,40-strlen(txt));
  } else {
    sprintf(txt," %1.1f Psi ", Pressure);
    vfd.Print(txt,40-strlen(txt));
  }

  digitalWrite(HEATER_ENABLE, HEATER_OFF);

  if (Temperature < 102.0) {
    digitalWrite(VENT_SOLENOID, VALVE_OPEN);
  } 
}

void tickHeater(void) {
  static uint8_t heater_counter = 0;
  int heater;

  digitalWrite(LED6, !digitalRead(LED6));

  if (InCycle && (state == heating) || (state == sterilizing)) {
    pid.run();

    heater = pidOutput;  

    if (heater <=0) {
      heater = 0;
    }

    if (heater >= 100) {
      heater = 100;
    }

    if (heater > heater_counter) {
      digitalWrite(HEATER1, HEATER_ON);
      // digitalWrite(HEATER2, HEATER_ON);
      // digitalWrite(HEATER3, HEATER_ON);
    } else {
      digitalWrite(HEATER1, HEATER_OFF);
      // digitalWrite(HEATER2, HEATER_OFF);
      // digitalWrite(HEATER3, HEATER_OFF);
    }
    
  } else {
    heater = 0;
    digitalWrite(HEATER1, HEATER_OFF);
    // digitalWrite(HEATER2, HEATER_OFF);
    // digitalWrite(HEATER3, HEATER_OFF);
  }

  heater_counter++;

  if (heater_counter > 100) {
    heater_counter = 0;
  }
}

void tick1Sec(void) {
  digitalWrite(LED7, !digitalRead(LED7));

  if (state == heating || state == sterilizing || state == cooling) {
    Seconds++;
  } else {
    // Seconds = 0;
  }
}

//Tickers Timers 
Ticker  timer1Sec(&tick1Sec, 1000, 0, MILLIS);
Ticker  timer50mSec(&tickHeater, 50, 0, MILLIS);


void setup() {
  // put your setup code here, to run once:
    state = ready;

  bool res = tempSensor.begin(MAX31865_4WIRE);
  Serial3.begin(9600);
  // Serial3.print(("Test"));
  vfd.setSerialPort(&Serial3);
  vfd.Init();

  //LED
  //Initialize LEDS
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED_OC, OUTPUT);
  digitalWrite(LED0, LED_OFF);
  digitalWrite(LED1, LED_OFF);
  digitalWrite(LED2, LED_OFF);
  digitalWrite(LED3, LED_OFF);
  digitalWrite(LED4, LED_OFF);
  digitalWrite(LED5, LED_OFF);
  digitalWrite(LED6, LED_OFF);
  digitalWrite(LED7, LED_OFF);

  //Initialize Vent Solenoid
  pinMode(VENT_SOLENOID,OUTPUT);
  digitalWrite(VENT_SOLENOID,VALVE_OPEN);

  //Initialize Start ans Stop buttons
  pinMode(BTN_START, INPUT);
  pinMode(BTN_STOP, INPUT);
  digitalWrite(BTN_START,1);
  digitalWrite(BTN_STOP,1);

  pinMode(HEATER1,OUTPUT);
  // pinMode(HEATER2,OUTPUT);
  // pinMode(HEATER3,OUTPUT);
  pinMode(HEATER_ENABLE,OUTPUT);
  digitalWrite(HEATER1, HEATER_OFF);
  // digitalWrite(HEATER2, HEATER_OFF);
  // digitalWrite(HEATER3, HEATER_OFF);
  digitalWrite(HEATER_ENABLE, HEATER_OFF);

  //Load settings form EEPROM
  Read_EEPROM();

}

void loop() {
  char txt[50];

  //Get the pressure sensor
  Read_Pressure();


  Read_Temperature();

  // char fValue[10];
  // dtostrf(avgVolts.GetAverage(), 5, 3, fValue);
  // sprintf(txt,"V:%s", fValue);
  // dtostrf(Pressure, 5,2, fValue);
  // sprintf(txt, "%s, P:%s  ", txt, fValue);
  // vfd.Print(txt,0);

  // //sprintf(txt,"Resist: %1.2f, Temp: %1.2f  ", Resistance, /* tempSensor.temperature(RNOMINAL, RREF) */ 2.12 );
  // dtostrf(Resistance, 5, 2, fValue);
  // sprintf(txt,"R:%s", fValue);
  // dtostrf(avgTemp.GetAverage(), 5,2, fValue);
  // sprintf(txt, "%s, T:%s  ", txt, fValue);
  // vfd.Print(txt, 20);
  

  digitalWrite(LED_OC, !digitalRead(LED_OC));
  //delay(100);
  
  timer1Sec.update();
  timer50mSec.update();

  // digitalWrite(LED5, !digitalRead(LED5));
  if ((digitalRead(BTN_START) == 0) && (state == ready)) {
    state = heating;
    Seconds = 0;
    // MinutesSetpoint = 0;
    uint32_t tstart= millis();
    while((millis()-tstart) < 1000) {
      //
    }

    InCycle = true;

    digitalWrite(HEATER_ENABLE, HEATER_ON);
    timer50mSec.start();
  }

  //if ((digitalRead(BTN_STOP) == 0) && (state == heating || state == sterilizing || state == cooling)) {
  if ((digitalRead(BTN_STOP) == 0) && (state == heating || state == sterilizing || state == cooling)) {
    state = ready;
    digitalWrite(VENT_SOLENOID,VALVE_OPEN);
    
    timer50mSec.stop();
    digitalWrite(LED6,LED_OFF);
    digitalWrite(HEATER1, HEATER_OFF);
    // digitalWrite(HEATER2, HEATER_OFF);
    // digitalWrite(HEATER3, HEATER_OFF);
    digitalWrite(HEATER_ENABLE, HEATER_OFF);
    
    digitalWrite(HEATER_LED, digitalRead(HEATER1));

    InCycle = false;

    vfd.Print("CICLO INTERRUMPIDO  ", 0);
    vfd.Print("POR USUARIO         ", 20);
    uint32_t tstart= millis();
    while((millis()-tstart) < 5000) {
      //
    }

    vfd.Init();
  }

  //Different states changing
  if (state == heating && (Temperature >= (Setpoint-1.0f))) {
    Seconds = 0;
    state = sterilizing;
  }

  if (state == sterilizing && (Seconds >= (MinutesSetpoint * 60))) {
    state = cooling;
    // timer50mSec.stop();
    Seconds = 0;
  }

  if (state == cooling && (Temperature<70.0f)) {
    state = ready;
    Seconds = 0;
  }

  //Selecte function to use depending on state
  switch (state) {
    case ready:
      Standby();
      break;
    case heating:
      Heating();
      break;
    case sterilizing:
      Sterilizing();
      break;
    case cooling:
      Cooling();
      break;
    default:
      Standby();
      break;
  }


  digitalWrite(HEATER_LED, digitalRead(HEATER1));
  digitalWrite(VALVE_LED, digitalRead(VENT_SOLENOID));
}

