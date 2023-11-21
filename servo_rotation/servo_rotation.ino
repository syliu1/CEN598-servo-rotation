#include <Servo.h>

#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace {
  const tflite::Model* model = nullptr; // Set TensorFlow model pointer
  tflite::MicroInterpreter* interpreter = nullptr; // Set TensorFlow interpreter pointer
  TfLiteTensor* input = nullptr; // Get pointer for model input tensor
  TfLiteTensor* output = nullptr; // Get pointer for model output tensor

  constexpr int kTensorArenaSize = 136 * 1024; // Need enough bytes for model
  alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}

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
    servoMotor.attach(SERVO_PIN);
    start_time = millis();
    initTensorFlow();
}

void loop() {
    readMics();
    findSignalMaxMinAllMics();

    now = millis();
    if ((now - start_time) >= (sampleWindow * 1000)) {
        findSignalAllMics();
        getServoAngle();
        start_time = millis();
  }
}

/**
* This function initializes everything needed to run the TensorFlow Lite model
*/
void initTensorFlow() {
  tflite::InitializeTarget();

  // Construct model from byte array
  model = tflite::GetModel(servoModel);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Get all operations
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run model with
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from tensor_arena for model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Get pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);
}

void findSignalMaxMinAllMics() {
    for(int i=0; i < NUMBER_MICROPHONES; i++) {
        findSignalMaxMin(i);
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

void findSignalAllMics() {
    for(int i=0; i < NUMBER_MICROPHONES; i++) {
        findSignal(i);
        signalMax[i] = SIGNAL_MIN;
        signalMin[i] = SIGNAL_MAX;
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

void setModelInput() {
    input->data.f[0] = yn_current[0];
    input->data.f[1] = yn_current[1];
    input->data.f[2] = yn_current[2];
    input->data.f[3] = yn_current[3];
    input->data.f[4] = yn_current[4];
    input->data.f[5] = yn_current[5];
}

void getServoAngle() {
    setModelInput(); // Update input to model

    TfLiteStatus invoke_status = interpreter->Invoke(); // Run inference
    if (invoke_status != kTfLiteOk) {
        Serial.println("Invoke Failed");
        return;
    }

    const float servoAngle = output->data.f[0]; // Get regression result
    handleServo(servoAngle); // Rotate servo
}

void handleServo(const float servoAngle) {
    servoMotor.write(servoAngle);
}