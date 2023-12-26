/*
  ESP32_DTSU666_Modbus_MQTT.h
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// modbus error code indexes
#define MODBUS_ERROR_CODE_0XE0_INDEX 0
#define MODBUS_ERROR_CODE_0XE1_INDEX 1
#define MODBUS_ERROR_CODE_0XE2_INDEX 2
#define MODBUS_ERROR_CODE_0XE3_INDEX 3

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// smart meter DTSU666 register blocks
#define DTSU666_REGISTER_BLOCK_DATA0   0
#define DTSU666_REGISTER_BLOCK_DATA1   1
#define DTSU666_REGISTER_BLOCK_DATA2   2
#define DTSU666_REGISTER_BLOCK_DATA3   3
#define DTSU666_REGISTER_BLOCK_DATA4   4

// smart meter DTSU666 Data0 [0x2000-0x2021 / 8192-8225] / register block #0
#define DTSU666_REGISTER_DATA0_HEAD                    0x2000
#define DTSU666_REGISTER_DATA0_TAIL                    0x2021
#define DTSU666_REGISTER_DATA0_SIZE                    (DTSU666_REGISTER_DATA0_TAIL - DTSU666_REGISTER_DATA0_HEAD + 1)
#define DTSU666_REGISTER_DATA0_ELEMENTS                16
#define DTSU666_REGISTER_DATA0_ACTIVE                  1
#define DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_A         0x2006
#define DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_B         0x2008
#define DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_C         0x200A
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
#define DTSU666_REGISTER_DATA3_ACTIVE                    0
#define DTSU666_REGISTER_DATA3_TOTAL_ACTIVE_POWER_DEMAND 0x2050

// smart meter DTSU666 Data4 [0x401E-0x4059 / 16414-16472] / register block #4
#define DTSU666_REGISTER_DATA4_HEAD                 0x401E
#define DTSU666_REGISTER_DATA4_TAIL                 0x4059
#define DTSU666_REGISTER_DATA4_SIZE                 (DTSU666_REGISTER_DATA4_TAIL - DTSU666_REGISTER_DATA4_HEAD + 1)
#define DTSU666_REGISTER_DATA4_ELEMENTS             12
#define DTSU666_REGISTER_DATA4_ACTIVE               1
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP     0x401E
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_A   0x4020
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_B   0x4022
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_C   0x4024
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_NET 0x4026
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP     0x4028
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_A   0x402A
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_B   0x402C
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_C   0x402E
#define DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_NET 0x4030

#define DTSU666_REGISTER_BLOCK_COUNT 5  // number of dtsu666 register blocks

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS 7
#define DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENT_LENGTH 64

const char dtsu666ModbusConverterMQTTSubscriptionsArray[DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS][DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENT_LENGTH] = {
  "DTSU666_Modbus_Converter_Modbus_Errors_0xE0",
  "DTSU666_Modbus_Converter_Modbus_Errors_0xE1",
  "DTSU666_Modbus_Converter_Modbus_Errors_0xE2",
  "DTSU666_Modbus_Converter_Modbus_Errors_0xE3",
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
  { DTSU666_REGISTER_DATA0_HEAD, DTSU666_REGISTER_DATA0_TAIL, DTSU666_REGISTER_DATA0_SIZE, DTSU666_REGISTER_DATA0_ELEMENTS, DTSU666_REGISTER_DATA0_ACTIVE },  // #0
  { DTSU666_REGISTER_DATA1_HEAD, DTSU666_REGISTER_DATA1_TAIL, DTSU666_REGISTER_DATA1_SIZE, DTSU666_REGISTER_DATA1_ELEMENTS, DTSU666_REGISTER_DATA1_ACTIVE },  // #1
  { DTSU666_REGISTER_DATA2_HEAD, DTSU666_REGISTER_DATA2_TAIL, DTSU666_REGISTER_DATA2_SIZE, DTSU666_REGISTER_DATA2_ELEMENTS, DTSU666_REGISTER_DATA2_ACTIVE },  // #2
  { DTSU666_REGISTER_DATA3_HEAD, DTSU666_REGISTER_DATA3_TAIL, DTSU666_REGISTER_DATA3_SIZE, DTSU666_REGISTER_DATA3_ELEMENTS, DTSU666_REGISTER_DATA3_ACTIVE },  // #3
  { DTSU666_REGISTER_DATA4_HEAD, DTSU666_REGISTER_DATA4_TAIL, DTSU666_REGISTER_DATA4_SIZE, DTSU666_REGISTER_DATA4_ELEMENTS, DTSU666_REGISTER_DATA4_ACTIVE }   // #4
};

struct dtsu666RegisterDataStruct {
	uint16_t dtsu666RegisterAddress;
  char *dtsu666RegisterDescriptor;
  char *dtsu666RegisterDataType;
  float dtsu666RegisterDataFactor;
  char *dtsu666RegisterUnit;
  bool dtsu666RegisterActiveMQTT;
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData0[] = {                                        // register block #0
  { DTSU666_REGISTER_DATA0_HEAD, "Block_Head", "", 0, "", 0 },                                            // #0
  { DTSU666_REGISTER_DATA0_TAIL, "Block_Tail", "", 0, "", 0 },                                            // #1
  { DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_A, "Voltage_Phase_A", "F32", 0.1, "V", 1 },                      // #2
  { DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_B, "Voltage_Phase_B", "F32", 0.1, "V", 1 },                      // #3
  { DTSU666_REGISTER_DATA0_VOLTAGE_PHASE_C, "Voltage_Phase_C", "F32", 0.1, "V", 1 },                      // #4
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_A, "Current_Phase_A", "F32", 0.001, "A", 1 },                    // #5
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_B, "Current_Phase_B", "F32", 0.001, "A", 1 },                    // #6
  { DTSU666_REGISTER_DATA0_CURRENT_PHASE_C, "Current_Phase_C", "F32", 0.001, "A", 1 },                    // #7
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_COMBINED, "Active_Power", "F32", 0.0001, "kW", 1 },               // #8
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_A, "Active_Power_Phase_A", "F32", 0.0001, "kW", 0 },        // #9
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_B, "Active_Power_Phase_B", "F32", 0.0001, "kW", 0 },        // #10
  { DTSU666_REGISTER_DATA0_ACTIVE_POWER_PHASE_C, "Active_Power_Phase_C", "F32", 0.0001, "kW", 0 },        // #11
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_COMBINED, "Reactive_Power", "F32", 0.0001, "kvar", 1 },         // #12
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_A, "Reactive_Power_Phase_A", "F32", 0.0001, "kvar", 0 },  // #13
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_B, "Reactive_Power_Phase_B", "F32", 0.0001, "kvar", 0 },  // #14
  { DTSU666_REGISTER_DATA0_REACTIVE_POWER_PHASE_C, "Reactive_Power_Phase_C", "F32", 0.0001, "kvar", 0 }   // #15
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData1[] = {                               // register block #1
  { DTSU666_REGISTER_DATA1_HEAD, "Block_Head", "", 0, "", 0 },                                   // #0
  { DTSU666_REGISTER_DATA1_TAIL, "Block_Tail", "", 0, "", 0 },                                   // #1
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_COMBINED, "Power_Factor", "F32", 0.001, "", 1 },         // #2
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_A, "Power_Factor_Phase_A", "F32", 0.001, "", 0 },  // #3
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_B, "Power_Factor_Phase_B", "F32", 0.001, "", 0 },  // #4
  { DTSU666_REGISTER_DATA1_POWER_FACTOR_PHASE_C, "Power_Factor_Phase_C", "F32", 0.001, "", 0 }   // #5
};
static struct dtsu666RegisterDataStruct dtsu666RegisterData2[] = {         // register block #2
  { DTSU666_REGISTER_DATA2_HEAD, "Block_Head", "", 0, "", 0 },             // #0
  { DTSU666_REGISTER_DATA2_TAIL, "Block_Tail", "", 0, "", 0 },             // #1
  { DTSU666_REGISTER_DATA2_FREQUENCY, "Frequency", "F32", 0.01, "Hz", 1 }  // #2
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData3[] = {                                           // register block #3
  { DTSU666_REGISTER_DATA3_HEAD, "Block_Head", "", 0, "", 0 },                                               // #0
  { DTSU666_REGISTER_DATA3_TAIL, "Block_Tail", "", 0, "", 0 },                                               // #1
  { DTSU666_REGISTER_DATA3_TOTAL_ACTIVE_POWER_DEMAND, "Total_Active_Power_Demand", "F32", 0.0001, "kW", 0 }  // #2
};

static struct dtsu666RegisterDataStruct dtsu666RegisterData4[] = {                             // register block #4
  { DTSU666_REGISTER_DATA4_HEAD, "Block_Head", "", 0, "", 0 },                                 // #0
  { DTSU666_REGISTER_DATA4_TAIL, "Block_Tail", "", 0, "", 0 },                                 // #1
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP, "Import_Energy_Total", "F32", 1.0, "kWh", 1 },    // #2
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_A, "Import_Energy_A", "F32", 1.0, "kWh", 0 },      // #3
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_B, "Import_Energy_B", "F32", 1.0, "kWh", 0 },      // #4
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_C, "Import_Energy_C", "F32", 1.0, "kWh", 0 },      // #5
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_IMP_NET, "Import_Energy_Net", "F32", 1.0, "kWh", 0 },  // #6
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP, "Export_Energy_Total", "F32", 1.0, "kWh", 1 },    // #7
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_A, "Export_Energy_A", "F32", 1.0, "kWh", 0 },      // #8
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_B, "Export_Energy_B", "F32", 1.0, "kWh", 0 },      // #9
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_C, "Export_Energy_C", "F32", 1.0, "kWh", 0 },      // #10
  { DTSU666_REGISTER_DATA4_TOTAL_ENERGY_EXP_NET, "Export_Energy_Net", "F32", 1.0, "kWh", 0 }   // #11
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void esp32Restart();

void serialOTAReceiver(uint8_t *data, size_t length);

void mqttOutputConverterState();

void outputInfoline();

void mqttKeepAliveReceiver(String &inputTopic, String &inputPayload);

void outputDTSU666Data(uint16_t dtsu666EntityResponseBufferOffset, char *dtsu666EntityDesriptor, char *dtsu666EntityDataType, float dtsu666EntityDataFactor, char *dtsu666EntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS);

bool mqttPublishTopicPayloadCHAR(String mqttTopicID, String mqttTopicEntity, char mqttPayloadCHAR, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt16(String mqttTopicID, String mqttTopicEntity, uint16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadUInt32(String mqttTopicID, String mqttTopicEntity, uint32_t mqttPayloadUInt32, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat16(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat16, bool mqttRetained, uint16_t mqttQoS);
bool mqttPublishTopicPayloadFloat32(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat32, bool mqttRetained, uint16_t mqttQoS);

//EOF