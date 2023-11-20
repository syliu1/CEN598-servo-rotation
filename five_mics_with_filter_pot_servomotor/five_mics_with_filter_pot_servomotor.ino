#include <Servo.h>

const int sensorPin1 = A0;  // Analog input pin
const int sensorPin2 = A1;
const int sensorPin3 = A2; 
const int sensorPin4 = A3;
const int sensorPin5 = A4;
const int potPin1 = A5;

Servo servoMotor1;

boolean filterFlag = 1;

int potReading;
int servoMotorAngle=0;

const int numberOfMicrophones = 5;

int mic[numberOfMicrophones];

float peakToPeak[numberOfMicrophones];
float volts[numberOfMicrophones];

const float sampleWindow = 0.05;

int signalMax[numberOfMicrophones] = {0, 0, 0, 0, 0};
int signalMin[numberOfMicrophones] = {1024, 1024, 1024, 1024, 1024};
float xn[numberOfMicrophones] = {0.0, 0.0, 0.0, 0.0, 0.0};
float yn_current[numberOfMicrophones] = {0.0, 0.0, 0.0, 0.0, 0.0};
float xn1[numberOfMicrophones] = {0.0, 0.0, 0.0, 0.0, 0.0};
float yn1[numberOfMicrophones] = {0.0, 0.0, 0.0, 0.0, 0.0};


unsigned long now;
unsigned long start_time;


void setup() {
  Serial.begin(115200);
  servoMotor1.attach(12);
  start_time = millis();
}

void findSignalMaxMin(int micNumber)
{
  if (mic[micNumber] < 1024) 
  {
    if (mic[micNumber] > signalMax[micNumber]) 
    {
      signalMax[micNumber] = mic[micNumber];
    } 
    else if (mic[micNumber] < signalMin[micNumber]) 
    {
      signalMin[micNumber] = mic[micNumber];
    }
  }
}

void findSignal(int micNumber)
{
  peakToPeak[micNumber] = signalMax[micNumber] - signalMin[micNumber];
  //volts[micNumber] = (peakToPeak[micNumber] * 3.3) / 1024.0;
  //xn[micNumber] = volts[micNumber];
  xn[micNumber] = peakToPeak[micNumber];
  
  // Apply your filter coefficients
  if(filterFlag == 1)
  {
    yn_current[micNumber] = 0.52188555 * yn1[micNumber] + 0.23905722  * xn[micNumber] + 0.23905722 * xn1[micNumber];
    xn1[micNumber] = xn[micNumber];
    yn1[micNumber] = yn_current[micNumber];
  }
  else
  {
    yn_current[micNumber] = peakToPeak[micNumber];
  }
}

void loop()
{
  mic[0] = analogRead(sensorPin1);
  mic[1] = analogRead(sensorPin2);
  mic[2] = analogRead(sensorPin3);
  mic[3] = analogRead(sensorPin4);
  mic[4] = analogRead(sensorPin5);
  
  potReading = analogRead(potPin1);
  servoMotorAngle = map(potReading, 0, 1023, 0, 180);
  servoMotor1.write(servoMotorAngle);
  
  for(int i=0; i<numberOfMicrophones; i++)
  {
    findSignalMaxMin(i);
  }

  now = millis();
  if ((now - start_time) >= (sampleWindow * 1000)) 
  {
    for(int i=0; i<numberOfMicrophones; i++)
    {
      findSignal(i);
      signalMax[i] = 0;
      signalMin[i] = 1024;
    }
    Serial.print(yn_current[0]);
    Serial.print(",");
    Serial.print(yn_current[1]);
    Serial.print(",");
    Serial.print(yn_current[2]);
    Serial.print(",");
    Serial.print(yn_current[3]);
    Serial.print(",");
    Serial.print(yn_current[4]);
    Serial.print(",");
    Serial.println(servoMotorAngle);
    start_time = millis();
  } 
}
