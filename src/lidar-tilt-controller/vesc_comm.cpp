#include <CAN.h>
#include <Arduino.h>

#include "vesc_typedefs.cpp"

#define VESC_CAN_ID 61
#define THIS_CAN_ID 5

class VescComm {

  private:
    int responsesLength = 0;

    uint8_t readBuffer[50];
    uint8_t readBufferLength = 0;
    uint8_t readBufferInfo[8];
    uint8_t readBufferInfoLength = 0;

  public:
    uint8_t fwVersionMajor = 0;
    uint8_t fwVersionMinor = 0;

    int32_t erpm;
    float_t current;
    float_t duty;

    float_t tempFet;
    float_t tempMotor;
    float_t currentIn;
    float_t pidPosNow;


  void setup() {
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
  }


  void getFwVersion() {    
    CAN.beginExtendedPacket(
      VESC_CAN_ID | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8)
    );
    CAN.write(THIS_CAN_ID);
    CAN.write(0x00);
    CAN.write(COMM_FW_VERSION);
    CAN.endPacket();
  }

  bool parseFwVersion() {
    uint8_t comm_fw_version = readBuffer[0];
    fwVersionMajor = readBuffer[1];
    fwVersionMinor = readBuffer[2];
    return true;
  }

  void printBuffer() {
    for(int i = 0; i < readBufferLength; i++) {
      char buffer[2];
      sprintf (buffer, "%02x", readBuffer[i]);
      Serial.print(buffer);
    }
    Serial.println();
  }

  void initRequest() {
    // Clear buffer
    readBufferLength = 0;
    for(int i = 0; i < 50; i++) {
      readBuffer[i] = 0; //(Not really necessary ?)
    }
    readBufferInfoLength = 0;
    for(int i = 0; i < 8; i++) {
      readBufferInfo[i] = 0; //(Not really necessary ?)
    }
  }

  void processCommandBuffer() {
    uint8_t command = readBuffer[0];
    if (command == COMM_FW_VERSION) {
      fwVersionMajor = readBuffer[1];
      fwVersionMinor = readBuffer[2];
      printBuffer();
    }

  }

  void processStatusPacket(uint8_t vescId, uint8_t commandId, uint8_t data[8]) {
    // Some good documentation on these packets can be found at
    //    https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can#1.-vesc-can-message-structure

    int32_t ind = 0;
    if (commandId == CAN_PACKET_STATUS) {
      // rpm(4 byte), current*10.0(2 byte), duty*1000.0(2 byte)
      erpm = bufferGetInt32(data, &ind);
      current = static_cast<float>(bufferGetInt16(data, &ind)) / 10.0;
      duty = static_cast<float>(bufferGetInt16(data, &ind)) / 1000.0;
    } else if (commandId == CAN_PACKET_STATUS_2) {
      // amp_hours*10000.0(4 byte), amp_hours_charged*10000.0(4 byte)
      // TODO - implement me
    } else if (commandId == CAN_PACKET_STATUS_3) {
      // watt_hours*10000.0(4 byte), watt_hours_charged*10000.0(4 byte)
      // TODO - implement me
    } else if (commandId == CAN_PACKET_STATUS_4) {
      // temp_fet*10.0(2 byte), temp_motor*10.0(2 byte), current_in*10.0(2 byte), pid_pos_now*50.0(2 byte)
      tempFet = static_cast<float>(bufferGetInt16(data, &ind)) / 10.0;
      tempMotor = static_cast<float>(bufferGetInt16(data, &ind)) / 10.0;
      currentIn = static_cast<float>(bufferGetInt16(data, &ind)) / 10.0;
      pidPosNow = static_cast<float>(bufferGetInt16(data, &ind)) / 50.0;
    } else if (commandId == CAN_PACKET_STATUS_5) {
      // tacho_value(4 byte), v_in*10.0(2 byte), reserved as 0(2 byte)
      // TODO - implement me
    }
  }

  void onReceive(int packetSize) {
    bool infoRecieved = false;
    long can_id = CAN.packetId();
    if (CAN.packetExtended()){
      if(can_id ==  ((uint16_t)CAN_PACKET_FILL_RX_BUFFER << 8) + THIS_CAN_ID) {
        int packetPosition = CAN.read();
        for (int i=0; i<packetSize-1; i++) {
          readBuffer[readBufferLength + i] = CAN.read();
        }
        readBufferLength = readBufferLength + packetSize - 1;
      } else if(can_id ==  ((uint16_t)CAN_PACKET_PROCESS_RX_BUFFER << 8) + THIS_CAN_ID) {
        for (int i=0; i<packetSize; i++) {
          readBufferInfo[readBufferInfoLength + i] = CAN.read();
        }
        readBufferInfoLength = readBufferInfoLength + packetSize;
        infoRecieved = true;

        uint16_t supposedLength = ((uint16_t)readBufferInfo[2] << 8) + readBufferInfo[3];
        if(readBufferLength != supposedLength){
          readBufferLength = 0;
          readBufferInfoLength = 0;
        } else {
          processCommandBuffer();
        }
      }
      else {
        // then it's a status message
        // these are automatically sent out from the VESC based on a frequency
        // in the settings
        u_int8_t vescId = can_id&0x000000ff;
        u_int8_t commandId = (can_id&0x0000ff00)>>8;
        u_int8_t data[8];

        for (int i=0; i<packetSize; i++) {
          uint8_t pv = CAN.read();
          data[i] = pv;
        }
        processStatusPacket(vescId, commandId, data);
      }
    }
  }

  int16_t bufferGetInt16(const uint8_t *buffer, int32_t *index) {
    int16_t res = ((uint16_t) buffer[*index]) << 8 | ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
  }

  int32_t bufferGetInt32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t) buffer[*index]) << 24 |
            ((uint32_t) buffer[*index + 1]) << 16 |
            ((uint32_t) buffer[*index + 2]) << 8 |
            ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
  }

};


