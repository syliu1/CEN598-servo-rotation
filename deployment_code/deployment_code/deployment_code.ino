// include the required TensorFlowLite libraries
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

// load the trained model weights
#include "model.h"

#include <Servo.h>

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* POSITIONS[] = {
  "0 degree",
  "36 degree",
  "72 degree",
  "108 degree",
  "144 degree",
  "180 degree",
  "silence"
};

#define NUM_OF_POSITIONS 7

const int sensorPin1 = A0;  // Analog input pin
const int sensorPin2 = A1;
const int sensorPin3 = A2; 
const int sensorPin4 = A3;
const int sensorPin5 = A4;

Servo servoMotor1;

boolean filterFlag = 1;

int potReading;
int previousServoMotorAngle=-1;

int previousPredictionClass = -1;
int predCount = 0;
int servoControlFlag = 0;

const int numberOfMicrophones = 5;
const int numberOfDataPerMic = 2;
const int inputSize = numberOfMicrophones*numberOfDataPerMic;

int classes[6] = {0, 36 ,72, 108, 144, 180}; 

int micDataCount = -1;

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
float input_array[inputSize] = {};

int prediction_class = -1;

unsigned long now;
unsigned long start_time;

int current_position = 90;


void setup() {
  Serial.begin(9600);
  servoMotor1.attach(12);
  servoMotor1.write(0);

  for(int i=0; i<90; i++)
      {
      servoMotor1.write(i);
      delay(50);
      }

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
  
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
    //yn_current[micNumber] = 0.88176521 * yn1[micNumber] + 0.0591174 * xn[micNumber] + 0.0591174 * xn1[micNumber];  // 20Hz cut-off frequency
    yn_current[micNumber] = 0.93908194 * yn1[micNumber] + 0.03045903 * xn[micNumber] + 0.03045903 * xn1[micNumber]; // 10Hz cut-off frequency
    //yn_current[micNumber] = 0.96906992 * yn1[micNumber] + 0.01546504 * xn[micNumber] + 0.01546504 * xn1[micNumber]; // 5Hz cut-off frequency
    xn1[micNumber] = xn[micNumber];
    yn1[micNumber] = yn_current[micNumber];
  }
  else
  {
    yn_current[micNumber] = peakToPeak[micNumber];
  }
}

void runInference()
{
  float max_prediction_value = -100;
  for(int i=0; i<inputSize; i++){
    tflInputTensor->data.f[i] = input_array[i] / 1023.0;
  }

  // Run inferencing
  TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  if (invokeStatus != kTfLiteOk) {
    Serial.println("Invoke failed!");
    while (1);
    return;
  }

  // Loop through the output tensor values from the model and find the class with highest prediction value
  //prediction_class=0;
  for (int i = 0; i < NUM_OF_POSITIONS; i++) 
  {
    if(tflOutputTensor->data.f[i]>max_prediction_value)
    {
      max_prediction_value = tflOutputTensor->data.f[i];
      prediction_class = i;
      //Serial.print("prediction_class: ");
      //Serial.println(prediction_class);
    }
  }
  Serial.print(POSITIONS[prediction_class]);
  Serial.print(": ");
  Serial.println(tflOutputTensor->data.f[prediction_class]);
  Serial.println();
  if(previousPredictionClass == prediction_class && prediction_class!=6){
    previousPredictionClass = prediction_class;
    predCount = predCount+1;
  }
  else{
    previousPredictionClass = prediction_class;
    predCount = 0;
  }
  if(predCount == 10){
    servoControlFlag = 1;
  }
  else{
    servoControlFlag = 0;
  }
}

void loop()
{
  mic[0] = analogRead(sensorPin1);
  mic[1] = analogRead(sensorPin2);
  mic[2] = analogRead(sensorPin3);
  mic[3] = analogRead(sensorPin4);
  mic[4] = analogRead(sensorPin5);

  if(servoControlFlag){
    if(classes[prediction_class]>current_position){
      for(int i=current_position; i<classes[prediction_class]; i++)
      {
      servoMotor1.write(i);
      delay(50);
      }
      current_position = classes[prediction_class];
    }
    else{
      for(int i=current_position; i>classes[prediction_class]; i--)
      {
      servoMotor1.write(i);
      delay(50);
      }
      current_position = classes[prediction_class];
    }
    Serial.print(classes[prediction_class]);
  }
  
  for(int i=0; i<numberOfMicrophones; i++)
  {
    findSignalMaxMin(i);
  }

  now = millis();
  if ((now - start_time) >= (sampleWindow * 1000)) 
  {
    if(micDataCount<(numberOfDataPerMic-1))
    {
      micDataCount++;
    }
    else{
      for(int k=0; k<inputSize; k++){
        Serial.print(input_array[k]);
        Serial.print(",");
      }
      Serial.println(" ");
      runInference();
      //delay(5000);
      micDataCount = 0;
    }
    for(int i=0; i<numberOfMicrophones; i++)
    {
      findSignal(i);
      input_array[(micDataCount + (i*numberOfDataPerMic))] = yn_current[i];
      signalMax[i] = 0;
      signalMin[i] = 1024;
    }
    start_time = millis();
  } 
}
