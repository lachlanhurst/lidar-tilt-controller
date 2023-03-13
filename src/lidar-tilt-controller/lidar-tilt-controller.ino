#include <Wire.h>
#include <CAN.h>

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

  // CAN setup
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
  
  // start the CAN bus at 500 kbps
  // this was 250k, but this may have an issue
  // see here https://github.com/adafruit/arduino-CAN/issues/3
  // VESC also defaults to 500k
  if (!CAN.begin(500000)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

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

