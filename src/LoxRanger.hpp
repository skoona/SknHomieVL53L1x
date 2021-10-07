/**
 * Homie Node for VL53L1x.
 *
 */

#pragma once

#include <Homie.hpp>
#include <VL53L1X.h>

class LoxRanger : public HomieNode {

public:
  LoxRanger(const char *id, const char *name, const char *cType, const unsigned long durationInSeconds, const int gpioPin);

  bool isReady();
  void setRunDuration(const int seconds);
  void startRanging();
  void stopRanging();

protected:
  virtual void setup() override;
  virtual void loop() override;
  virtual void onReadyToOperate() override;
  virtual bool handleInput(const HomieRange &range, const String &property, const String &value) override;

private:
  int _pinGPIO;
  unsigned long _ulCycleTime;

  const char *cCaption = "• VL53L1x Ranging Module:";
  const char *cIndent = "  ◦ ";

  const char *cRangeID = "range";
  const char *cRangeFormat = "%04d";
  const char *cStatusID = "status";
  const char *cStatusFormat = "%s";
  const char *cSignalID = "signal";
  const char *cSignalFormat = "%04.2f";
  const char *cAmbientID = "ambient";
  const char *cAmbientFormat = "%03.2f";

  unsigned long ulTimebase = 0,
                ulLastTimebase = 0,
                ulCycleTimebase = 0,
                ulRangingDuration = 280,
                ulElapsedTime = 0;

  volatile bool vbRangeDuration = false,
                vbLastRangeCycle = true,
                vbRangeCycle = false,
                vbEnabled = false;

  void printCaption();

  VL53L1X lox;

};
