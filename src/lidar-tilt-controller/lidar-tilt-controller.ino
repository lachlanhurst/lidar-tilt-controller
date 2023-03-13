#include <Wire.h>

#include "oled.h"
#include "vesc_comm.cpp"
#include "wiring_private.h"


TwoWire myWire(&sercom1, 5, 25);

OLED display(OLED::W_128,OLED::H_64);

uint32_t timestamp;

VescComm vescComm;

void setup()
{
  // Begin serial debug
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN VESC demo");

  // setup display
  myWire.begin();
  myWire.setClock(1600000L);
  // Assign pins 5 & 25 to SERCOM functionality
  // for 2c display
  pinPeripheral(5, PIO_SERCOM);
  pinPeripheral(25, PIO_SERCOM);
  // Begin display
  display.begin(myWire);

  vescComm.setup();

  timestamp = millis();

  display.clear();
  display.draw_string(0, 0, "Starting up...");
  display.display();
}


void loop()
{
  if (!vescComm.fwVersionMajor) {
    Serial.println("Sending packet get firmware version");
    // then read the firmware
    vescComm.getFwVersion();

    char szTemp[16];
    sprintf(szTemp, "VESC FW %d.%d", vescComm.fwVersionMajor, vescComm.fwVersionMinor);
    Serial.println(szTemp);
    display.clear();
    display.draw_string(0, 0, szTemp);
    display.display();

    delay(500);
  }


}

