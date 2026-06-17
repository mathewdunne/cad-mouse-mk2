#include "controllers/TelemetryController.h"

#include <Arduino.h>
#include <string.h>

#include "Config.h"
#include "Controllers.h"

namespace {
const int kPrintEvery = 5;
const unsigned long kRawPrintIntervalMs = 10;

bool isCommandSpace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}
}  // namespace

void TelemetryController::begin() {
  tick_ = 0;
  rawModeActive_ = false;
  lastRawPrintMs_ = 0;
  commandLength_ = 0;
}

bool TelemetryController::serialEnabled() const {
  return Config::ENABLE_TELEMETRY || Config::ENABLE_CALIBRATION_SERIAL;
}

bool TelemetryController::enabled() const {
  return Config::ENABLE_TELEMETRY && !rawModeActive_;
}

bool TelemetryController::rawModeActive() const { return rawModeActive_; }

void TelemetryController::update() {
  if (!serialEnabled()) {
    return;
  }

  while (Serial.available() > 0) {
    processSerialByte(static_cast<char>(Serial.read()));
  }
}

void TelemetryController::processSerialByte(char c) {
  if (c == '\r' || c == '\n') {
    if (commandLength_ > 0) {
      commandBuffer_[commandLength_] = '\0';
      handleCommand();
      commandLength_ = 0;
    }
    return;
  }

  if (commandLength_ >= (kCommandBufferSize - 1)) {
    commandLength_ = 0;
    return;
  }

  commandBuffer_[commandLength_++] = c;
}

void TelemetryController::handleCommand() {
  char* start = commandBuffer_;
  while (*start != '\0' && isCommandSpace(*start)) {
    start++;
  }

  char* end = start + strlen(start);
  while (end > start && isCommandSpace(*(end - 1))) {
    *(--end) = '\0';
  }

  if (strcmp(start, "CAL?") == 0) {
    replyStatus();
    return;
  }

  if (strcmp(start, "CAL RAW 1") == 0) {
    rawModeActive_ = true;
    lastRawPrintMs_ = 0;
    Serial.println("CAL,RAW,1");
    return;
  }

  if (strcmp(start, "CAL RAW 0") == 0) {
    rawModeActive_ = false;
    lastRawPrintMs_ = 0;
    Serial.println("CAL,RAW,0");
    return;
  }

  if (strncmp(start, "CAL", 3) == 0) {
    Serial.println("CAL,ERR,UNKNOWN");
  }
}

void TelemetryController::replyStatus() const {
  Serial.print("CAL,STATUS,ready=");
  Serial.print(sensorController.calibrationDone() ? 1 : 0);
  Serial.print(",raw=");
  Serial.print(rawModeActive_ ? 1 : 0);
  Serial.print(",baseline=");
  Serial.print(sensorController.calibrationDone() ? 1 : 0);
  Serial.print(",decoder=");
  Serial.println(motionController.usingCalibratedDecoder() ? 1 : 0);
}

void TelemetryController::publish(const float motion[6], int buttonBits,
                                  bool hidReportSent) {
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  Serial.print(">X:");
  Serial.println(motion[0]);
  Serial.print(">Y:");
  Serial.println(motion[1]);
  Serial.print(">Z:");
  Serial.println(motion[2]);
  Serial.print(">Rx:");
  Serial.println(motion[3]);
  Serial.print(">Ry:");
  Serial.println(motion[4]);
  Serial.print(">Rz:");
  Serial.println(motion[5]);
  Serial.print(">btn:");
  Serial.println(buttonBits & 0x0003);
  Serial.print(">hid:");
  Serial.println(hidReportSent ? 1 : 0);
}

void TelemetryController::publishRaw(const float raw[9]) {
  if (!Config::ENABLE_CALIBRATION_SERIAL || !rawModeActive_) {
    return;
  }

  const unsigned long now = millis();
  if (lastRawPrintMs_ != 0 &&
      (now - lastRawPrintMs_) < kRawPrintIntervalMs) {
    return;
  }
  lastRawPrintMs_ = now;

  Serial.print("CALRAW,");
  Serial.print(now);
  for (int i = 0; i < 9; i++) {
    Serial.print(",");
    Serial.print(raw[i], 6);
  }
  Serial.println();
}
