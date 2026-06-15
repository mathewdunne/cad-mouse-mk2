#include "states/SleepState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void SleepState::enter() {
  ledController.off();
}

void SleepState::update() {
  inputController.update();

  if (telemetryController.rawModeActive()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }

  if (inputController.takeActivity()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }
}

void SleepState::exit() {}
