#include <Wire.h>
#include "Arduino.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "globals.h"
#include "declares.h"
#include "display.h"
#include "settings.h"

void PrintMenu()
{
  Serial.println("1: Device Status");
  Serial.println("2: Voltage Reading");
  Serial.println("3: ...");
  Serial.println("4: ...");
  Serial.println("r: ReScan for Devices");
}

void setupDisplay()
{
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);
}

void oled_loop()
{
  // Control what is currently being displayed
  //
  oledDisplay = DISPLAYSTATUS;
  switch (oledDisplay)
  {
    case DISPLAYMENU:
      displayMenu(); // Displays the menu on screen for user to select option
      break;
    case DISPLAYSTATUS:
      displayStatus(); // Displays the status in intervals of 'displayStatusRefresh'
      break;
    case DISPLAYVOLTAGES:
      break;
    default:
      oledDisplay = DISPLAYSTATUS;
      break;
  }
}

void displayMenu()
{
  oled.println("1: Device Status");
  oled.println("2: Voltage Reading");
  oled.println("3: ...");
  oled.println("4: ...");
  oled.println("r: ReScan for Devices");
}

void displayStatus()
{
  static long previousTimer;
  unsigned long currentTimer = millis();
  static uint8_t currentDisplay;

  if (currentTimer - previousTimer > settings.displayStatusRefresh) // rebuild the picture after some delay
  {
    previousTimer = currentTimer;
    oled.clear();
    oled.set1X();
    oled.println("     System Status");
    switch (currentDisplay)
    {
      case 1:
        displaySysStatus();
        break;
      case 2:
        displayVoltage();
        break;
      case 3:
        //        displayCondition();
        break;
      case 4:
        break;
      default:
        currentDisplay = 0;
        break;
    }
    currentDisplay += 1;
  }
}

void printScreen(uint8_t x, uint8_t y, String input)
{
  char *buf;
  input.toCharArray(buf, input.length());
  //  oled.drawStr( x, y, buf);
}

void displaySysStatus()
{
  //  oled.println("Hello world!");
  //  oled.println("A long line may be truncated");
  //  oled.println();
  //  oled.set2X();
  //  oled.println("2X demo");
  //  oled.set1X();

  oled.println( "SS not available");
}

void displayVoltage()
{
//  oled.println( "Cell Voltages:");
//  for (int i = 0; i < 4; i++) {
//    bms.vc_hi[i] = readFrom(deviceAddress, VC_HI[i], 1);
//    bms.vc_lo[i] = readFrom(deviceAddress, VC_LO[i], 1);
//  }

  oled.println( "1.10 | 2.20 | 3.30");
  oled.println( "4.40");
}

