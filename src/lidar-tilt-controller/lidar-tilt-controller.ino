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
}


void loop()
{
  display.draw_string(0, 0, "foo bar");
  display.draw_pixel(0,0);
  display.draw_pixel(127,0);
  display.display();
  
  if ((millis() - timestamp) > 9000) {
    Serial.println("Sending packet get firmware version");
    
    vescComm.getFwVersion();

    Serial.println("...sent!");
    timestamp = millis();
  }

}

