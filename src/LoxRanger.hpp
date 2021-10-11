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
  unsigned long setDuration(unsigned long duration) ;

private : 
  int _pinGPIO;
  unsigned long _ulCycleTime;
  unsigned int uiDistanceValue = 0;
  #define MAX_SAMPLES 5
  unsigned int distances[MAX_SAMPLES + 2];
  char cDirection[32]; // CLOSED, OPEN, ClOSING, OPENING

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
  const char *cOperateID = "ranging";
  const char *cOperateFormat = "ON,OFF,ERROR";
  const char *cDirectionID = "direction";
  const char *cDirectionFormat = "CLOSING,OPENING,IDLE";


  unsigned long ulTimebase = 0,             // current ms count
                ulLastTimebase = 0,         // ms from last operation
                ulCycleDuration = 30000,    // time window to operate
                ulRangingDuration = 280,    // distance read time
                ulElapsedTime = 0;          // elapsed from timebase

  volatile bool vbRangeDuration = false,    // time to read lox
                vbLastRangeCycle = true,    // lox off signal
                vbRunCycle = false,         // run top level for ulCycleDuration
                vbEnabled = false;          // operating trigger

  void printCaption();
  unsigned int handleLoxRead();
  VL53L1X lox;

};
