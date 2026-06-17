#pragma once

class TelemetryController {
 public:
  void begin();
  void update();
  void publish(const float motion[6], int buttonBits, bool hidReportSent);
  void publishRaw(const float raw[9]);
  bool enabled() const;
  bool rawModeActive() const;

 private:
  static const int kCommandBufferSize = 48;

  bool serialEnabled() const;
  void processSerialByte(char c);
  void handleCommand();
  void replyStatus() const;

  int tick_ = 0;
  bool rawModeActive_ = false;
  unsigned long lastRawPrintMs_ = 0;
  char commandBuffer_[kCommandBufferSize] = {};
  int commandLength_ = 0;
};
