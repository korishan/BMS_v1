#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "globals.h"
#include "declares.h"
#include "display.h"
#include "settings.h"

#include <bq769x0.h>    // Library for Texas Instruments bq76920 battery management IC

SSD1306AsciiWire oled;
userSettings settings;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  int err = bmsBegin();

  setupDisplay();
  setDefaults();
  PrintMenu();
}

void loop() {
  unsigned long currentTimer = millis();
  static unsigned long bmsUpdateLast;
  static unsigned long serialLastUpdate;

  if (currentTimer - bmsUpdateLast > settings.bmsRefreshInt)
  {
    bmsUpdate();  // should be called at least every 250 ms
    bmsUpdateLast = currentTimer;
  }
  if (currentTimer - serialLastUpdate > settings.displayStatusRefresh)
  {
    //BMS.printRegisters();
    Serial.print(F("Cell Voltages: "));
    for (int i = 1; i <= 5; i++)
    {
      Serial.print(getCellVoltage(i));
      Serial.print(F(" "));
    }
    Serial.println();
    serialLastUpdate = currentTimer;
  }
  readSerial(); // Check serial input for commands
}

void setDefaults()
{
  setBMSDefaults();

  settings.displayStatusRefresh = 5000;
  settings.bmsRefreshInt = 250; // Later we will pull this from a user stored variable
  settings.oledDisplay = DISPLAYSTATUS;
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
