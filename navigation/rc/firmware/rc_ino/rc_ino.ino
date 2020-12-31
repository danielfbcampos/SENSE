#include <EnableInterrupt.h>
#include "Arduino.h"

#define rcPin1 8   // Pin 8 Connected to CH1 of Transmitter;
#define rcPin2 9   // Pin 9 Connected to CH2
#define rcPin3 10   // Pin 9 Connected to CH2
#define rcPin4 11
#define rcPin5 12   // Pin 12 Connected to CH9
#define rele 4

uint16_t rc_values[5];
uint32_t rc_start[5];
volatile uint16_t rc_shared[5];

int ch1 = 0;  // Receiver Channel 1 PPM value
int ch2 = 0;  // Receiver Channel 2 PPM value
int ch3 = 0;
int ch4 = 0;
int ch5 = 0;


int ch1_min = 1500;
int ch1_max = 1500;
int ch2_min = 1500;
int ch2_max = 1500;

bool first_calib=false;

int ch1_after;

int safety=0;
int rc=0;

float lin_x = 0;
float ang_z = 0;

String incomingString;

bool read_calib = false;

bool calib = false;

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(0, rcPin1); }
void calc_ch2() { calc_input(1, rcPin2); }
void calc_ch3() { calc_input(2, rcPin3); }
void calc_ch4() { calc_input(3, rcPin4); }
void calc_ch5() { calc_input(4, rcPin5); }

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void setup() {
  pinMode(rcPin1, INPUT);
  pinMode(rcPin2, INPUT);
  pinMode(rcPin3, INPUT);
  pinMode(rcPin4, INPUT);
  pinMode(rcPin5, INPUT);
  pinMode(rele, OUTPUT);  

  enableInterrupt(rcPin1, calc_ch1, CHANGE);
  enableInterrupt(rcPin2, calc_ch2, CHANGE);
  enableInterrupt(rcPin3, calc_ch3, CHANGE);
  enableInterrupt(rcPin4, calc_ch4, CHANGE);
  enableInterrupt(rcPin5, calc_ch5, CHANGE);
  
  Serial.begin(9600);
}

void loop() {

  if(!read_calib){ 
    if ( Serial.available() > 0) {
      incomingString = Serial.readString();
    

      ch1_min = getValue(incomingString,',',0).toInt();
      ch1_max = getValue(incomingString,',',1).toInt();
      ch2_min = getValue(incomingString,',',2).toInt();
      ch2_max = getValue(incomingString,',',3).toInt(); 

      read_calib =true;
    }

    Serial.print("RC");
    Serial.print(",");
    Serial.print(read_calib);  
    Serial.println();
  }
  else{
  // Read in the length of the signal in microseconds
    rc_read_values();
    
    //ch1 = pulseIn(rcPin1, HIGH, 30000);  // (Pin, State, Timeout)
    //ch2 = pulseIn(rcPin2, HIGH, 30000);
    //ch3 = pulseIn(rcPin3, HIGH, 30000);
    //ch4 = pulseIn(rcPin4, HIGH, 30000);
    //ch5 = pulseIn(rcPin5, CHANGE, 30000);
  
    ch1 = rc_values[0];  
    ch2 = rc_values[1];  
    ch3 = rc_values[2];
    ch4 = rc_values[3];
    ch5 = rc_values[4];
  
    if(ch3<1500) safety=0;
    if(ch3>=1500) safety=1;
  
    if (safety==0) digitalWrite(rele, HIGH);
    if (safety==1) digitalWrite(rele, LOW);
  
    if(ch5 >= 1500){
      if(!first_calib){
        ch1_min = 1500;
        ch1_max = 1500;
        ch2_min = 1500;
        ch2_max = 1500;
        first_calib=true;
      }
      calib = true;
    }
    else
    {
      calib = false;
      first_calib = false;
    }
    
    
    if(calib)
    {
      if(ch1>ch1_max)
        ch1_max = ch1;
  
      if(ch2>ch2_max)
        ch2_max = ch2;
  
      if(ch1<ch1_min)
        ch1_min = ch1;
  
      if(ch2<ch2_min)
        ch2_min = ch2; 
    }
  
    int ch1_m = map(ch1, ch1_min, ch1_max, 1000, 2000);
    int ch2_m = map(ch2, ch2_min, ch2_max, 1000, 2000);
    
  
    if(!calib){
      Serial.print(ch1_m);
      Serial.print(",");
      Serial.print(ch2_m);
      Serial.print(",");
      Serial.print(safety);
      Serial.print(",");
      Serial.print(ch4);   
      Serial.println();
    }
    else{
      Serial.print("C");
      Serial.print(",");
      Serial.print(ch1_min);
      Serial.print(",");
      Serial.print(ch1_max);
      Serial.print(",");
      Serial.print(ch2_min);
      Serial.print(",");
      Serial.print(ch2_max);   
      Serial.println();
    }
  } 
}
