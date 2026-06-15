#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
HIDController hidController;
TelemetryController telemetryController;

void setup() {
  // Initialize USB HID first
  hidController.begin();

  if (Config::ENABLE_TELEMETRY || Config::ENABLE_CALIBRATION_SERIAL) {
    Serial.begin(115200);
    delay(200);
  }

  inputController.begin();
  ledController.begin();
  sensorController.begin();
  motionController.reset();
  telemetryController.begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void loop() {
  hidController.task();
  telemetryController.update();
  stateMachine.update();
}
