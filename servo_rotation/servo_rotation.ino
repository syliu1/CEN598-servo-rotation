#include <Servo.h>

const int SENSOR_PIN_ONE       = A0;  // Analog input pin
const int SENSOR_PIN_TWO       = A1;
const int SENSOR_PIN_THREE     = A2; 
const int SENSOR_PIN_FOUR      = A3;
const int SENSOR_PIN_FIVE      = A4;
const int SERVO_PIN            = 12;
const int BAUD_RATE            = 115200;
const int NUMBER_MICROPHONES   = 5;
const int SIGNAL_MIN           = 0;
const int SIGNAL_MAX           = 1024;
const float SAMPLE_WINDOW      = 0.05;
const float FILTER_COEFF_ONE   = 0.52188555;
const float FILTER_COEFF_TWO   = 0.23905722;

int mic[NUMBER_MICROPHONES];

float peakToPeak[NUMBER_MICROPHONES];
float volts[NUMBER_MICROPHONES];

int signalMax[NUMBER_MICROPHONES]    = {0, 0, 0, 0, 0};
int signalMin[NUMBER_MICROPHONES]    = {1024, 1024, 1024, 1024, 1024};
float xn[NUMBER_MICROPHONES]         = {0.0, 0.0, 0.0, 0.0, 0.0};
float yn_current[NUMBER_MICROPHONES] = {0.0, 0.0, 0.0, 0.0, 0.0};
float xn1[NUMBER_MICROPHONES]        = {0.0, 0.0, 0.0, 0.0, 0.0};
float yn1[NUMBER_MICROPHONES]        = {0.0, 0.0, 0.0, 0.0, 0.0};

Servo servoMotor;

boolean filterFlag  = 1;

int servoMotorAngle = 0;
int potReading;

unsigned long now;
unsigned long start_time;

void setup() {
    Serial.begin(BAUD_RATE);
    servoMotor1.attach(SERVO_PIN);
    start_time = millis();
}

void loop() {
    readMics();

    for(int i=0; i < NUMBER_MICROPHONES; i++) {
        findSignalMaxMin(i);
    }

    now = millis();
    if ((now - start_time) >= (sampleWindow * 1000)) {
        for(int i=0; i < NUMBER_MICROPHONES; i++) {
            findSignal(i);
            signalMax[i] = SIGNAL_MIN;
            signalMin[i] = SIGNAL_MAX;
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
        start_time = millis();
  }
}

void findSignalMaxMin(const int micNumber) {
    if (mic[micNumber] < 1024) {
        if (mic[micNumber] > signalMax[micNumber]) {
            signalMax[micNumber] = mic[micNumber];
        } else if (mic[micNumber] < signalMin[micNumber]) {
            signalMin[micNumber] = mic[micNumber];
        }
    }
}

void findSignal(const int micNumber) {
    peakToPeak[micNumber] = signalMax[micNumber] - signalMin[micNumber];
    xn[micNumber] = peakToPeak[micNumber];

    // Apply your filter coefficients
    if(filterFlag == 1) {
        yn_current[micNumber] = 
            FILTER_COEFF_ONE * yn1[micNumber] + 
            FILTER_COEFF_TWO * xn[micNumber]  + 
            FILTER_COEFF_TWO * xn1[micNumber];
        xn1[micNumber] = xn[micNumber];
        yn1[micNumber] = yn_current[micNumber];
    } else {
        yn_current[micNumber] = peakToPeak[micNumber];
    }
}

void readMics() {
    mic[0] = analogRead(SENSOR_PIN_ONE);
    mic[1] = analogRead(SENSOR_PIN_TWO);
    mic[2] = analogRead(SENSOR_PIN_THREE);
    mic[3] = analogRead(SENSOR_PIN_FOUR);
    mic[4] = analogRead(SENSOR_PIN_FIVE);
}