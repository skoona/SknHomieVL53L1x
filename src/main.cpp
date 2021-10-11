#include <Arduino.h>
#include <Homie.h>
#include <Wire.h>

#include <LoxRanger.hpp>


#define SKN_MOD_NAME "VL53L1x Tof Ranger"
#define SKN_MOD_VERSION "1.0.4"
#define SKN_MOD_BRAND "SknSensors"

#define SKN_NODE_TITLE "VL53L1x Tof Ranger"
#define SKN_NODE_TYPE "measurement"
#define SKN_NODE_ID "position"

#define LOX_RUNTIME_SECONDS 30
#define LOX_PIN_SDA 21
#define LOX_PIN_SCL 22
#define LOX_PIN_GPIO 19

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

HomieSetting<long> cfgDuration("duration", "Seconds to measure distance after triggered.");

LoxRanger ranger(SKN_NODE_ID, SKN_NODE_TITLE, SKN_NODE_TYPE, LOX_RUNTIME_SECONDS, LOX_PIN_GPIO);

bool broadcastHandler(const String &level, const String &value)
{
    Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;

    return true;
}

void setup()
{
    Serial.begin(115200);

    Wire.begin(LOX_PIN_SDA, LOX_PIN_SCL, 400000U);

    Homie_setFirmware(SKN_MOD_NAME, SKN_MOD_VERSION);
    Homie_setBrand(SKN_MOD_BRAND);

    cfgDuration
        .setDefaultValue(20)
        .setValidator([](long candidate)
                      { return candidate > 0 && candidate < 181; });

    ranger.setRunDuration(cfgDuration.get());

    Homie.setBroadcastHandler(broadcastHandler)
        .setLedPin(LED_BUILTIN, LOW)
        .disableResetTrigger();

    Homie.setup();
}

void loop()
{
    Homie.loop();
}