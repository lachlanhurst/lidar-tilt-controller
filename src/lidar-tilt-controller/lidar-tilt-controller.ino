#include <Wire.h>

#include "oled.h"
#include "vesc_comm.cpp"
#include "wiring_private.h"


TwoWire myWire(&sercom1, 5, 25);

OLED display(OLED::W_128,OLED::H_64);

uint32_t timestamp;

VescComm vescComm;

bool requestedFwVersion = false;
bool running = false;

void onReceive(int packetSize) {
  vescComm.onReceive(packetSize);
}

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

  CAN.onReceive(onReceive);
}


void loop()
{
  if (running) {
    display.clear();
    char sTemp[32];
    sprintf(sTemp, "erpm: %d", vescComm.erpm);
    display.println(sTemp);
    sprintf(sTemp, "current: %0.1f", vescComm.current);
    display.println(sTemp);
    sprintf(sTemp, "Temp: f %0.1f, m %0.1f", vescComm.tempFet, vescComm.tempMotor);
    display.println(sTemp);

    display.display();
  } else if (!vescComm.fwVersionMajor && !requestedFwVersion) {
    Serial.println("Sending packet get firmware version");
    // then read the firmware
    vescComm.initRequest();
    vescComm.getFwVersion();
    requestedFwVersion = true;
  } else if (requestedFwVersion && vescComm.fwVersionMajor) {
    char szTemp[16];
    sprintf(szTemp, "VESC FW %d.%d", vescComm.fwVersionMajor, vescComm.fwVersionMinor);
    Serial.println(szTemp);
    display.clear();
    display.draw_string(0, 0, szTemp);
    display.display();

    delay(500);
    requestedFwVersion = false;
    running = true;
  }

  // Serial.print("temp fet = ");
  // Serial.println(vescComm.tempFet);
  // Serial.print("temp motor = ");
  // Serial.println(vescComm.tempMotor);

  delay(200);
}

