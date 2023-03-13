#include <CAN.h>
#include <Arduino.h>

#define VESC_CAN_ID 61
#define THIS_CAN_ID 5

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING,
	COMM_GPD_SET_FSW,
	COMM_GPD_BUFFER_NOTIFY,
	COMM_GPD_BUFFER_SIZE_LEFT,
	COMM_GPD_FILL_BUFFER,
	COMM_GPD_OUTPUT_SAMPLE,
	COMM_GPD_SET_MODE,
	COMM_GPD_FILL_BUFFER_INT8,
	COMM_GPD_FILL_BUFFER_INT16,
	COMM_GPD_SET_BUFFER_INT_SCALE,
	COMM_GET_VALUES_SETUP,
	COMM_SET_MCCONF_TEMP,
	COMM_SET_MCCONF_TEMP_SETUP,
	COMM_GET_VALUES_SELECTIVE,
	COMM_GET_VALUES_SETUP_SELECTIVE,
	COMM_EXT_NRF_PRESENT,
	COMM_EXT_NRF_ESB_SET_CH_ADDR,
	COMM_EXT_NRF_ESB_SEND_DATA,
	COMM_EXT_NRF_ESB_RX_DATA,
	COMM_EXT_NRF_SET_ENABLED,
	COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
	COMM_DETECT_APPLY_ALL_FOC,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
	COMM_ERASE_NEW_APP_ALL_CAN,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN,
	COMM_PING_CAN,
	COMM_APP_DISABLE_OUTPUT,
	COMM_TERMINAL_CMD_SYNC,
	COMM_GET_IMU_DATA,
	COMM_BM_CONNECT,
	COMM_BM_ERASE_FLASH_ALL,
	COMM_BM_WRITE_FLASH,
	COMM_BM_REBOOT,
	COMM_BM_DISCONNECT,
	COMM_BM_MAP_PINS_DEFAULT,
	COMM_BM_MAP_PINS_NRF5X,
	COMM_ERASE_BOOTLOADER,
	COMM_ERASE_BOOTLOADER_ALL_CAN,
	COMM_PLOT_INIT,
	COMM_PLOT_DATA,
	COMM_PLOT_ADD_GRAPH,
	COMM_PLOT_SET_GRAPH,
	COMM_GET_DECODED_BALANCE,
	COMM_BM_MEM_READ,
	COMM_WRITE_NEW_APP_DATA_LZO,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
	COMM_BM_WRITE_FLASH_LZO,
	COMM_SET_CURRENT_REL,
	COMM_CAN_FWD_FRAME,
	COMM_SET_BATTERY_CUT,
	COMM_SET_BLE_NAME,
	COMM_SET_BLE_PIN,
	COMM_SET_CAN_MODE,
	COMM_GET_IMU_CALIBRATION,
	COMM_GET_MCCONF_TEMP,

	// Custom configuration for hardware
	COMM_GET_CUSTOM_CONFIG_XML,
	COMM_GET_CUSTOM_CONFIG,
	COMM_GET_CUSTOM_CONFIG_DEFAULT,
	COMM_SET_CUSTOM_CONFIG,

	// BMS commands
	COMM_BMS_GET_VALUES,
	COMM_BMS_SET_CHARGE_ALLOWED,
	COMM_BMS_SET_BALANCE_OVERRIDE,
	COMM_BMS_RESET_COUNTERS,
	COMM_BMS_FORCE_BALANCE,
	COMM_BMS_ZERO_CURRENT_OFFSET,

	// FW updates commands for different HW types
	COMM_JUMP_TO_BOOTLOADER_HW,
	COMM_ERASE_NEW_APP_HW,
	COMM_WRITE_NEW_APP_DATA_HW,
	COMM_ERASE_BOOTLOADER_HW,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW,
	COMM_ERASE_NEW_APP_ALL_CAN_HW,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW,
	COMM_ERASE_BOOTLOADER_ALL_CAN_HW,

	COMM_SET_ODOMETER,

	// Power switch commands
	COMM_PSW_GET_STATUS,
	COMM_PSW_SWITCH,

	COMM_BMS_FWD_CAN_RX,
	COMM_BMS_HW_DATA,
	COMM_GET_BATTERY_CUT,
	COMM_BM_HALT_REQ,
	COMM_GET_QML_UI_HW,
	COMM_GET_QML_UI_APP,
	COMM_CUSTOM_HW_DATA,
	COMM_QMLUI_ERASE,
	COMM_QMLUI_WRITE,

	// IO Board
	COMM_IO_BOARD_GET_ALL,
	COMM_IO_BOARD_SET_PWM,
	COMM_IO_BOARD_SET_DIGITAL,

	COMM_BM_MEM_WRITE,
	COMM_BMS_BLNC_SELFTEST,
	COMM_GET_EXT_HUM_TMP,
	COMM_GET_STATS,
	COMM_RESET_STATS,

	// Lisp
	COMM_LISP_READ_CODE,
	COMM_LISP_WRITE_CODE,
	COMM_LISP_ERASE_CODE,
	COMM_LISP_SET_RUNNING,
	COMM_LISP_GET_STATS,
	COMM_LISP_PRINT,

	COMM_BMS_SET_BATT_TYPE,
	COMM_BMS_GET_BATT_TYPE,

	COMM_LISP_REPL_CMD,
	COMM_LISP_STREAM_CODE,

	COMM_FILE_LIST,
	COMM_FILE_READ,
	COMM_FILE_WRITE,
	COMM_FILE_MKDIR,
	COMM_FILE_REMOVE,

	COMM_LOG_START,
	COMM_LOG_STOP,
	COMM_LOG_CONFIG_FIELD,
	COMM_LOG_DATA_F32,

	COMM_SET_APPCONF_NO_STORE,
	COMM_GET_GNSS,

	COMM_LOG_DATA_F64,
} COMM_PACKET_ID;

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12,
	CAN_PACKET_IO_BOARD_DIGITAL_IN,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	CAN_PACKET_BMS_V_TOT,
	CAN_PACKET_BMS_I,
	CAN_PACKET_BMS_AH_WH,
	CAN_PACKET_BMS_V_CELL,
	CAN_PACKET_BMS_BAL,
	CAN_PACKET_BMS_TEMPS,
	CAN_PACKET_BMS_HUM,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,
	CAN_PACKET_PSW_STAT,
	CAN_PACKET_PSW_SWITCH,
	CAN_PACKET_BMS_HW_DATA_1,
	CAN_PACKET_BMS_HW_DATA_2,
	CAN_PACKET_BMS_HW_DATA_3,
	CAN_PACKET_BMS_HW_DATA_4,
	CAN_PACKET_BMS_HW_DATA_5,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL,
	CAN_PACKET_UPDATE_PID_POS_OFFSET,
	CAN_PACKET_POLL_ROTOR_POS,
	CAN_PACKET_NOTIFY_BOOT,
	CAN_PACKET_STATUS_6,
	CAN_PACKET_GNSS_TIME,
	CAN_PACKET_GNSS_LAT,
	CAN_PACKET_GNSS_LON,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP,
	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;


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


