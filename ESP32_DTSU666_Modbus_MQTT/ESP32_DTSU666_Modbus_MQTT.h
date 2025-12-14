/*
  ESP32_DTSU666_Modbus_MQTT.h
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// esp32 restart info
#define ESP32_RESTART_SYSTEM               0
#define ESP32_RESTART_WIFI_CONNECTION      1
#define ESP32_RESTART_MODBUS_KEEP_ALIVE    2
#define ESP32_RESTART_MQTT_CONNECTION      3
#define ESP32_RESTART_OTA_RESTART          4
#define ESP32_RESTART_OTA_HALT             5

#define ESP32_RESTART_INFO_ELEMENTS 6

struct esp32RestartCounterStruct {
  uint8_t esp32RestartID;
  char *esp32RestartReasonNVS;  // max. 15 characters!
  char *esp32RestartReason;
  uint32_t esp32RestartCounter;
};

static struct esp32RestartCounterStruct esp32RestartInfo[] = {
  { ESP32_RESTART_SYSTEM, "SYSTEM", "System", 0},
  { ESP32_RESTART_WIFI_CONNECTION, "WIFI", "WiFi Connection Error", 0},
  { ESP32_RESTART_MODBUS_KEEP_ALIVE, "MODBUS", "Modbus Keep Alive Error", 0},
  { ESP32_RESTART_MQTT_CONNECTION, "MQTT", "MQTT Connection Error", 0},
  { ESP32_RESTART_OTA_RESTART, "OTA_RESTART", "OTA Restart", 0},
  { ESP32_RESTART_OTA_HALT, "OTA_HALT", "OTA Halt", 0}
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// Modbus definitions
/* ModbudMaster Return Codes
  ku8MBSuccess          = 0x00;
  ku8MBInvalidSlaveID   = 0xE0;
  ku8MBInvalidFunction  = 0xE1;
  ku8MBResponseTimedOut = 0xE2;
  ku8MBInvalidCRC       = 0xE3;
 */
#define MODBUS_ERROR_CODE_OFFSET (0xE0 - 0x01)  // 0xDF

const char* modbusReturnCodes[] = {
  "Success",
  "Invalid Slave ID",
  "Invalid Dunction",
  "Response Timeout",
  "CRC Error",
};

// modbus return code indexes
#define MODBUS_SUCCESS                0
#define MODBUS_ERROR_INVALID_SLAVE_ID 1
#define MODBUS_ERROR_INVALID_FUNCTION 2
#define MODBUS_ERROR_RESPONCE_TIMEOUT 3
#define MODBUS_ERROR_INVALID_CRC      4

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// smart meter DTSU666 System [0x0000-0x002E / 0-46] / register block system
#define DTSU666_REGISTER_SYSTEM_HEAD                  0x0000
#define DTSU666_REGISTER_SYSTEM_TAIL                  0x002E
#define DTSU666_REGISTER_SYSTEM_SIZE                  (DTSU666_REGISTER_SYSTEM_TAIL - DTSU666_REGISTER_SYSTEM_HEAD + 1)
#define DTSU666_REGISTER_SYSTEM_ELEMENTS              14
#define DTSU666_REGISTER_SYSTEM_ACTIVE                1
#define DTSU666_REGISTER_SYSTEM_REVISION              0x0000
#define DTSU666_REGISTER_SYSTEM_UCODE                 0x0001
#define DTSU666_REGISTER_SYSTEM_POWER_RESET           0x0002
#define DTSU666_REGISTER_SYSTEM_WIRE_SELECTION        0x0003
#define DTSU666_REGISTER_SYSTEM_CURRENT_RATE          0x0006
#define DTSU666_REGISTER_SYSTEM_VOLTAGE_RATE          0x0007
#define DTSU666_REGISTER_SYSTEM_DISPLAY_ROTATION_TIME 0x000A
#define DTSU666_REGISTER_SYSTEM_LCD_BACKLIGHT_TIME    0x000B
#define DTSU666_REGISTER_SYSTEM_ENDIAN                0x000C
#define DTSU666_REGISTER_SYSTEM_PROTOCOL              0x002C
#define DTSU666_REGISTER_SYSTEM_RS485_BAUDRATE        0x002D
#define DTSU666_REGISTER_SYSTEM_MODBUS_ADDRESS        0x002E

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// smart meter DTSU666 Data0 [0x2006-0x2021 / 8198-8225] / register block #0
#define DTSU666_REGISTER_DATA0_HEAD                    0x2006
#define DTSU666_REGISTER_DATA0_TAIL                    0x2021
#define DTSU666_REGISTER_DATA0_SIZE                    (DTSU666_REGISTER_DATA0_TAIL - DTSU666_REGISTER_DATA0_HEAD + 1)
#define DTSU666_REGISTER_DATA0_ELEMENTS                16
#define DTSU666_REGISTER_DATA0_ACTIVE                  1
#define DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_A   0x2006
#define DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_B   0x2008
#define DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_C   0x200A
#define DTSU666_REGISTER_DATA0_CURRENT_PHASE_A         0x200C
#define DTSU666_REGISTER_DATA0_CURRENT_PHASE_B         0x200E
#define DTSU666_REGISTER_DATA0_CURRENT_PHASE_C         0x2010
#define DTSU666_REGISTER_DATA0_ACTIVE_POWER_COMBINED   0x2012
#define DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_A    0x2014
#define DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_B    0x2016
#define DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_C    0x2018
#define DTSU666_REGISTER_DATA0_REACTIVE_POWER_COMBINED 0x201A
#define DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_A  0x201C
#define DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_B  0x201E
#define DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_C  0x2020

// smart meter DTSU666 Data1 [0x202A-0x2031 / 8234-8241] / register block #1
#define DTSU666_REGISTER_DATA1_HEAD                  0x202A
#define DTSU666_REGISTER_DATA1_TAIL                  0x2031
#define DTSU666_REGISTER_DATA1_SIZE                  (DTSU666_REGISTER_DATA1_TAIL - DTSU666_REGISTER_DATA1_HEAD + 1)
#define DTSU666_REGISTER_DATA1_ELEMENTS              6
#define DTSU666_REGISTER_DATA1_ACTIVE                1
#define DTSU666_REGISTER_DATA1_POWER_FACTOR_COMBINED 0x202A
#define DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_A  0x202C
#define DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_B  0x202E
#define DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_C  0x2030

// smart meter DTSU666 Data2 [0x2044-0x2045 / 8260-8261] / register block #2
#define DTSU666_REGISTER_DATA2_HEAD      0x2044
#define DTSU666_REGISTER_DATA2_TAIL      0x2045
#define DTSU666_REGISTER_DATA2_SIZE      (DTSU666_REGISTER_DATA2_TAIL - DTSU666_REGISTER_DATA2_HEAD + 1)
#define DTSU666_REGISTER_DATA2_ELEMENTS  3
#define DTSU666_REGISTER_DATA2_ACTIVE    1
#define DTSU666_REGISTER_DATA2_FREQUENCY 0x2044

// smart meter DTSU666 Data3 [0x2050-0x2051 / 8272-8273] / register block #3
#define DTSU666_REGISTER_DATA3_HEAD                      0x2050
#define DTSU666_REGISTER_DATA3_TAIL                      0x2051
#define DTSU666_REGISTER_DATA3_SIZE                      (DTSU666_REGISTER_DATA3_TAIL - DTSU666_REGISTER_DATA3_HEAD + 1)
#define DTSU666_REGISTER_DATA3_ELEMENTS                  3
#define DTSU666_REGISTER_DATA3_ACTIVE                    1
#define DTSU666_REGISTER_DATA3_TOTAL_ACTIVE_POWER_DEMAND 0x2050

// smart meter DTSU666 Data4 [0x101E-0x1031 / 4126-4145] / register block #4
#define DTSU666_REGISTER_DATA4_HEAD                 0x101E
#define DTSU666_REGISTER_DATA4_TAIL                 0x1031
#define DTSU666_REGISTER_DATA4_SIZE                 (DTSU666_REGISTER_DATA4_TAIL - DTSU666_REGISTER_DATA4_HEAD + 1)
#define DTSU666_REGISTER_DATA4_ELEMENTS             12
#define DTSU666_REGISTER_DATA4_ACTIVE               1
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP     0x101E
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_A   0x1020
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_B   0x1022
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_C   0x1024
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_NET 0x1026
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP     0x1028
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_A   0x102A
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_B   0x102C
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_C   0x102E
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_NET 0x1030

/*
// smart meter DTSU666 Data5 [0x401E-0x4031 / 16414-16433] / register block #5
#define DTSU666_REGISTER_DATA5_HEAD                 0x401E
#define DTSU666_REGISTER_DATA5_TAIL                 0x4031
#define DTSU666_REGISTER_DATA5_SIZE                 (DTSU666_REGISTER_DATA5_TAIL - DTSU666_REGISTER_DATA5_HEAD + 1)
#define DTSU666_REGISTER_DATA5_ELEMENTS             12
#define DTSU666_REGISTER_DATA5_ACTIVE               1
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_IMP     0x401E
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_IMP_A   0x4020
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_IMP_B   0x4022
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_IMP_C   0x4024
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_IMP_NET 0x4026
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_EXP     0x4028
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_EXP_A   0x402A
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_EXP_B   0x402C
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_EXP_C   0x402E
#define DTSU666_REGISTER_DATA5_TOTAL_ENERGY_EXP_NET 0x4030
*/

#define DTSU666_REGISTER_BLOCK_COUNT 6  // number of dtsu666 register blocks

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define DTSU666_MODBUS_CONVERTER_MQTT_OUTPUT_ELEMENTS 7
#define DTSU666_MODBUS_CONVERTER_MQTT_OUTPUT_ELEMENT_LENGTH 64

const char dtsu666ModbusConverterMQTTOutputArray[DTSU666_MODBUS_CONVERTER_MQTT_OUTPUT_ELEMENTS][DTSU666_MODBUS_CONVERTER_MQTT_OUTPUT_ELEMENT_LENGTH] = {
  "DTSU666_Modbus_Converter_Error_0xE0",
  "DTSU666_Modbus_Converter_Error_0xE1",
  "DTSU666_Modbus_Converter_Error_0xE2",
  "DTSU666_Modbus_Converter_Error_0xE3",
  "DTSU666_Modbus_Converter_MQTT_Keep_Alive",
  "DTSU666_Modbus_Converter_MQTT_Keep_Alive_Received",
  "DTSU666_Modbus_Converter_MQTT_Keep_Alive_Delayed"
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
struct dtsu666RegisterBlockStruct {
  uint16_t dtsu666RegisterBlockHead;
  uint16_t dtsu666RegisterBlockTail;
  uint16_t dtsu666RegisterBlockSize;
  uint16_t dtsu666RegisterBlockElements;
  uint16_t dtsu666RegisterBlockActive;
};

static struct dtsu666RegisterBlockStruct dtsu666RegisterBlocks[] = {
  { DTSU666_REGISTER_SYSTEM_HEAD, DTSU666_REGISTER_SYSTEM_TAIL, DTSU666_REGISTER_SYSTEM_SIZE, DTSU666_REGISTER_SYSTEM_ELEMENTS, DTSU666_REGISTER_SYSTEM_ACTIVE },  // #0
  { DTSU666_REGISTER_DATA0_HEAD, DTSU666_REGISTER_DATA0_TAIL, DTSU666_REGISTER_DATA0_SIZE, DTSU666_REGISTER_DATA0_ELEMENTS, DTSU666_REGISTER_DATA0_ACTIVE },  // #1
  { DTSU666_REGISTER_DATA1_HEAD, DTSU666_REGISTER_DATA1_TAIL, DTSU666_REGISTER_DATA1_SIZE, DTSU666_REGISTER_DATA1_ELEMENTS, DTSU666_REGISTER_DATA1_ACTIVE },  // #2
  { DTSU666_REGISTER_DATA2_HEAD, DTSU666_REGISTER_DATA2_TAIL, DTSU666_REGISTER_DATA2_SIZE, DTSU666_REGISTER_DATA2_ELEMENTS, DTSU666_REGISTER_DATA2_ACTIVE },  // #3
  { DTSU666_REGISTER_DATA3_HEAD, DTSU666_REGISTER_DATA3_TAIL, DTSU666_REGISTER_DATA3_SIZE, DTSU666_REGISTER_DATA3_ELEMENTS, DTSU666_REGISTER_DATA3_ACTIVE },  // #4
  { DTSU666_REGISTER_DATA4_HEAD, DTSU666_REGISTER_DATA4_TAIL, DTSU666_REGISTER_DATA4_SIZE, DTSU666_REGISTER_DATA4_ELEMENTS, DTSU666_REGISTER_DATA4_ACTIVE }   // #5
};

struct dtsu666RegisterDataStruct {
	uint16_t dtsu666RegisterAddress;
  char *dtsu666RegisterDescriptor;
  char *dtsu666RegisterDataType;
  float dtsu666RegisterDataFactor;
  char *dtsu666RegisterUnit;
  bool dtsu666RegisterActiveMQTT;
};

static struct dtsu666RegisterDataStruct dtsu666RegisterSystem[] = {                               // register block system
  { DTSU666_REGISTER_SYSTEM_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_SYSTEM_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_SYSTEM_REVISION, "Software_Revision", "I16", 1.0, "", 1 },                   // #0
  { DTSU666_REGISTER_SYSTEM_UCODE, "Programming_Code", "I16", 1.0, "", 1 },                       // #1
  { DTSU666_REGISTER_SYSTEM_POWER_RESET, "Power_Reset", "I16", 1.0, "", 1 },                      // #2
  { DTSU666_REGISTER_SYSTEM_WIRE_SELECTION, "Wire_Selection", "I16", 1.0, "", 1 },                // #3
  { DTSU666_REGISTER_SYSTEM_CURRENT_RATE, "Transformer_Rate_IrAt", "I16", 1.0, "", 1 },           // #4
  { DTSU666_REGISTER_SYSTEM_VOLTAGE_RATE, "Transformer_Rate_IrUt", "I16", 1.0, "", 1 },           // #5
  { DTSU666_REGISTER_SYSTEM_DISPLAY_ROTATION_TIME, "Display_Rotation_Time", "I16", 1.0, "", 1 },  // #6
  { DTSU666_REGISTER_SYSTEM_LCD_BACKLIGHT_TIME, "LCD_Backlight_Time", "I16", 1.0, "", 1 },        // #7
  { DTSU666_REGISTER_SYSTEM_ENDIAN, "Reserve", "I16", 1.0, "", 1 },                               // #8
  { DTSU666_REGISTER_SYSTEM_PROTOCOL, "Protocol", "I16", 1.0, "", 1 },                            // #9
  { DTSU666_REGISTER_SYSTEM_RS485_BAUDRATE, "RS485_Baudrate", "I16", 1.0, "", 1 },                // #10
  { DTSU666_REGISTER_SYSTEM_MODBUS_ADDRESS, "Modbus_Address", "I16", 1.0, "", 1 }                 // #11
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData0[] = {                                        // register block #0
  { DTSU666_REGISTER_DATA0_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA0_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_A, "Voltage_Phase_A", "F32", 0.1, "V", 1 },                // #0
  { DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_B, "Voltage_Phase_B", "F32", 0.1, "V", 1 },                // #1
  { DTSU666_REGISTER_DATA0_PHASE_VOLTAGE_PHASE_C, "Voltage_Phase_C", "F32", 0.1, "V", 1 },                // #2
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_A, "Current_Phase_A", "F32", 0.001, "A", 1 },                    // #3
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_B, "Current_Phase_B", "F32", 0.001, "A", 1 },                    // #4
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_C, "Current_Phase_C", "F32", 0.001, "A", 1 },                    // #5
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_COMBINED, "Active_Power", "F32", 0.0001, "kW", 1 },               // #6
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_A, "Active_Power_Phase_A", "F32", 0.0001, "kW", 1 },        // #7
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_B, "Active_Power_Phase_B", "F32", 0.0001, "kW", 1 },        // #8
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_C, "Active_Power_Phase_C", "F32", 0.0001, "kW", 1 },        // #9
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_COMBINED, "Reactive_Power", "F32", 0.0001, "kVAR", 1 },         // #10
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_A, "Reactive_Power_Phase_A", "F32", 0.0001, "kVAR", 1 },  // #11
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_B, "Reactive_Power_Phase_B", "F32", 0.0001, "kVAR", 1 },  // #12
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_C, "Reactive_Power_Phase_C", "F32", 0.0001, "kVAR", 1 }   // #13
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData1[] = {                               // register block #1
  { DTSU666_REGISTER_DATA1_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA1_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_COMBINED, "Power_Factor", "F32", 0.001, "", 1 },         // #0
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_A, "Power_Factor_Phase_A", "F32", 0.001, "", 1 },  // #1
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_B, "Power_Factor_Phase_B", "F32", 0.001, "", 1 },  // #2
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_C, "Power_Factor_Phase_C", "F32", 0.001, "", 1 }   // #3
};
static struct dtsu666RegisterDataStruct dtsu666RegisterData2[] = {         // register block #2
  { DTSU666_REGISTER_DATA2_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA2_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA2_FREQUENCY, "Frequency", "F32", 0.01, "Hz", 1 }  // #0
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData3[] = {                                           // register block #3
  { DTSU666_REGISTER_DATA3_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA3_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA3_TOTAL_ACTIVE_POWER_DEMAND, "Total_Active_Power_Demand", "F32", 0.0001, "kW", 1 }  // #0
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData4[] = {                               // register block #4
  { DTSU666_REGISTER_DATA4_HEAD, "Block_Head", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA4_TAIL, "Block_Tail", "", 0, "", 0 },
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP, "Import_Energy_Total", "F32", 1.0, "kWh", 1 },      // #0
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_A, "Import_Energy_Phase_A", "F32", 1.0, "kWh", 1 },  // #1
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_B, "Import_Energy_Phase_B", "F32", 1.0, "kWh", 1 },  // #2
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_C, "Import_Energy_Phase_C", "F32", 1.0, "kWh", 1 },  // #3
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_NET, "Import_Energy_Net", "F32", 1.0, "kWh", 1 },    // #4
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP, "Export_Energy_Total", "F32", 1.0, "kWh", 1 },      // #5
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_A, "Export_Energy_Phase_A", "F32", 1.0, "kWh", 1 },  // #6
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_B, "Export_Energy_Phase_B", "F32", 1.0, "kWh", 1 },  // #7
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_C, "Export_Energy_Phase_C", "F32", 1.0, "kWh", 1 },  // #8
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_NET, "Export_Energy_Net", "F32", 1.0, "kWh", 1 }     // #9
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void esp32Restart(uint8_t esp32RestartID);

void serialOTAReceiver(uint8_t *data, size_t length);

void handleModbusErrorCounters(uint8_t resultModbus);

void mqttOutputConverterState();

void outputInfoline();

void mqttMessageReceiver(String &inputTopic, String &inputPayload);

void outputDTSU666Data(uint16_t dtsu666EntityResponseBufferOffset, char *dtsu666EntityDescriptor, char *dtsu666EntityDataType, float dtsu666EntityDataFactor, char *dtsu666EntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS);

bool mqttPublishTopicPayloadCHAR(String mqttTopicID, String mqttTopicEntity, char mqttPayloadCHAR, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadInt16(String mqttTopicID, String mqttTopicEntity, int16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt16(String mqttTopicID, String mqttTopicEntity, uint16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt32(String mqttTopicID, String mqttTopicEntity, uint32_t mqttPayloadUInt32, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat16(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat32(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat32, bool mqttRetained, uint16_t mqttQoS);

//EOF