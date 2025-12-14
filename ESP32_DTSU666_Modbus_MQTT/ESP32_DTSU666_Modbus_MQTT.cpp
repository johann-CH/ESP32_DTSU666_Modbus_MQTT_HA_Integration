/*
  ESP32 + RS485 Converter: Smart Meter DTSU666 Prod Home Assistant Integration (RS485 / Modbus RTU / MQTT)
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <ESP32.h>
#include <Preferences.h>

#include <Chrono.h>
#include <WiFi.h>
#include <Time.h>
#include <esp_sntp.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>

#include <ModbusMaster.h>
#include <MQTT.h>

#include "ESP32_DTSU666_Modbus_MQTT.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SERIAL_BAUDRATE 115200      // serial interface bauderate
#define SERIAL_DELAY 200            // serial communication delay [ms]
#define SERIAL_RX_BUFFER_SIZE 2048  // serial RX buffer size [byte]
#define SERIAL_TX_BUFFER_SIZE 2048  // serial TX buffer size [byte]

#ifdef ARDUINO_ESP32S3_DEV
  #define RS485 Serial1              // rename serial interface name
  #define RS485_BAUDRATE 9600        // serial interface bauderate
  #define RS485_MODE SERIAL_8N1      // serial mode (1 start / 8 data / 1 stop / no parity)
  #define RS485_IO_RX 18             // GPIO18:RX1 (ESP32)
  #define RS485_IO_TX 17             // GPIO17:TX1 (ESP32)
  #define RS485_DELAY 200            // RS485 communication delay [ms]
  #define RS485_RX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]
  #define RS485_TX_BUFFER_SIZE 2048  // RS485 TX buffer size [byte]
#endif

#ifdef ARDUINO_ESP32_DEV
  #define RS485 Serial2              // rename serial interface name
  #define RS485_BAUDRATE 9600        // serial interface bauderate
  #define RS485_MODE SERIAL_8N1      // serial mode (1 start / 8 data / 1 stop / no parity)
  #define RS485_IO_RX 16             // GPIO16:RX2 (ESP32)
  #define RS485_IO_TX 17             // GPIO17:TX2 (ESP32)
  #define RS485_DELAY 200            // RS485 communication delay [ms]
  #define RS485_RX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]
  #define RS485_TX_BUFFER_SIZE 2048  // RS485 TX buffer size [byte]
#endif

#define MODBUS_SLAVE_ID 0x01  // Modbus slave address ID: 0x01 Smart Meter DTSU666
#define MODBUS_DELAY 20       // Modbus communication delay [ms]

#define CONNECTION_DELAY 1000        // WiFi / MQTT connection delay [ms]
#define CONNECTION_RETRY_DELAY 1000  // WiFi / MQTT retry connection delay [ms]

#define MQTT_LOOP_DELAY 10       // MQTT loop communication delay [ms]
#define MQTT_SUBSCRIBE_DELAY 10  // MQTT subscribe communication delay [ms]
#define MQTT_PUBLISH_DELAY 10    // MQTT publish communication delay [ms]

#define MQTT_IO_BUFFER_SIZE 512  // MQTT I/O buffer size [byte]

#define MQTT_KEEP_ALIVE_INTERVAL_CHRONO 60  // chrono MQTT keep alive interval [s] / MQTT_KEEP_ALIVE_INTERVAL_CHRONO > MQTT_KEEP_ALIVE_DELAY_CHRONO!
#define MQTT_KEEP_ALIVE_DELAY_CHRONO 15     // chrono MQTT keep alive delay [s]
#define MQTT_KEEP_ALIVE_MAX_DELAYS 5        // chrono MQTT keep alive delays

#define DTSU666_DATA_HANDLE_INTERVAL_CHRONO 2  // chrono dtsu666 data handle interval [s]

#define STATE_OUTPUT_REFRESH_INTERVAL_CHRONO 10  // chrono dtsu666 + converter states output refresh interval [s]

#define DEBUG_OUTPUT  // default serial interface output active

#define OTA_OUTPUT  // over the air serial monitor output active

#define ESP32_RESTART_TIME "06:00"      // ESP32 restart time 06h00
#define ESP32_RESTART_LOCK_DELAY 60000  // ESP32 restart lock delay 60s [ms]
#define ESP32_RESTART_DELAY 5000        // ESP32 restart command delay [ms]
#define ESP32_HALT_DELAY 60000          // ESP32 halt delay 60s [ms]

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// instantiate preferences library
Preferences nvs;  // non-volatile storage (NVS)

// instantiate chrono objects
Chrono mqttKeepAliveChrono(Chrono::SECONDS);
Chrono mqttKeepAliveDelayChrono(Chrono::SECONDS);
Chrono dtsu666DataHandleChrono(Chrono::SECONDS);
Chrono refreshOutputStateChrono(Chrono::SECONDS);

// instantiate WiFi+NTP+MQTT objects
WiFiClient wifiConnectionID;
MQTTClient mqttClient(MQTT_IO_BUFFER_SIZE);

// instantiate Modbus object
ModbusMaster modbusNode;

// instantiate over the air serial monitor
#ifdef OTA_OUTPUT
  AsyncWebServer serialOTA(80);
#endif

// structure definition library <Time.h>
struct tm timeInfo;

// WiFi definitions
const char wifiSSID[] = "WLAN_Belp";  // WiFi network SSID
const char wifiPassword[] = "family.ruch@belp";  // WiFi network password

// SNTP definitions
const char *timeZone = "CET-1CEST,M3.5.0,M10.5.0/3";  // Time Zone rule for Europe/Rome including daylight adjustment rules (optional)
const char *sntpTimeServer1 = "ch.pool.ntp.org";
const char *sntpTimeServer2 = "pool.ntp.org";
const char *sntpServername = sntpTimeServer1;
u8_t sntpServerID = 0;
const uint32_t gmtOffset = 3600;       // GMT+1 [s]
const uint16_t daylightOffset = 3600;  // 1h [s]
String actDateTime = "1970-01-01 / 00:00:00";
String actTimestamp = "00:00";
String sntpTimeZone = "CET";  // "CET" or "CEST"

// Modbus definitions
unsigned int resultModbus;
unsigned long modbusErrorCounter = 0;
unsigned long modbusErrorCodes[5] = {0};

// MQTT definitions
const char mqttServer[] = "192.168.1.150";
int mqttServerPort = 1883;
const char mqttUser[] = "mqtt-admin";
const char mqttPassword[] = "mqtt-admin";
const char mqttClientID[] = "dtsu666_mqtt_prod";
bool mqttSuccess;
bool mqttDisconnectSkip = false;

String mqttTopic;
String mqttTopicID;
String mqttTopicEntity;
String mqttPayload;
unsigned long mqttPayloadData;
unsigned int mqttPayloadInt;
float mqttPayloadFloat;
bool mqttRetained = false;
const unsigned int mqttQoS = 0;

uint16_t dtsu666RegisterBlockHead;
uint16_t dtsu666RegisterBlockSize;
uint16_t dtsu666RegisterBlockElements;
uint16_t dtsu666RegisterBlockBaseAddress;
uint16_t dtsu666EntityRegisterAddress;
uint16_t dtsu666EntityResponseBufferOffset;
char *dtsu666EntityDescriptor;
char *dtsu666EntityDataType;
float dtsu666EntityDataFactor;
char *dtsu666EntityUnit;
bool dtsu666EntityActiveMQTT;
char bufferValueChar;
int16_t bufferValueInt16;
uint16_t bufferValueUInt16;
uint32_t bufferValueUInt32;

unsigned int mqttReconnectionCounter = 0;
const unsigned int mqttMaxReconnections = 10;

unsigned long mqttKeepAliveCounter = 0;
unsigned long mqttKeepAliveReceivedCounter = 0;
unsigned long mqttKeepAliveDelayedCounter = 0;

// common definitions
char msgString[256];  // message char array

bool esp32Halt = 0;
bool connectionsEstablished = 0;

String dtsu666RegisterBlockName;
unsigned int dtsu666RegisterBlockIndex = 0;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // initialize chronos
  mqttKeepAliveChrono.stop();
  mqttKeepAliveDelayChrono.stop();
  dtsu666DataHandleChrono.stop();
  refreshOutputStateChrono.stop();
  
  // initialize default serial interface
  Serial.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);
  Serial.setTxBufferSize(SERIAL_TX_BUFFER_SIZE);
  Serial.begin(SERIAL_BAUDRATE);
  delay(SERIAL_DELAY);
  Serial.flush();
  
  // initialize serial interface (RS485)
  RS485.setRxBufferSize(RS485_RX_BUFFER_SIZE);
  RS485.setTxBufferSize(RS485_TX_BUFFER_SIZE);
  RS485.begin(RS485_BAUDRATE, RS485_MODE, RS485_IO_RX, RS485_IO_TX);
  delay(RS485_DELAY);
  RS485.flush();
  
  // initialize NVS (R+W)
  nvs.begin("CTA_NVS", false);
  
  // read persisted NVS values
  for(uint8_t i = 0; i < ESP32_RESTART_INFO_ELEMENTS; i++) {
    esp32RestartInfo[i].esp32RestartCounter = nvs.getUInt(esp32RestartInfo[i].esp32RestartReasonNVS, esp32RestartInfo[i].esp32RestartCounter);
  }
  
  // output infoline
  Serial.println("Smart Meter DTSU666 Prod [Init Modbus + WiFi + MQTT]");
  Serial.println();
  
  // initalize modbus slave <ID>
  modbusNode.begin(MODBUS_SLAVE_ID, RS485);
  delay(MODBUS_DELAY);
  modbusNode.clearResponseBuffer();
  
  // initialize WiFi network connection
  Serial.print("WiFi connecting...");
  WiFi.mode(WIFI_STA);  // WiFi station mode
  WiFi.begin(wifiSSID, wifiPassword);
  delay(CONNECTION_DELAY);
  while(WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.disconnect();
    sprintf(msgString, "\nWiFi Error: WiFi not connected!");
    Serial.println(msgString);
    
    esp32Restart(ESP32_RESTART_WIFI_CONNECTION);
  }
  Serial.print("\nWiFi <");
  Serial.print(wifiSSID);
  Serial.print("> / <");
  Serial.print(WiFi.localIP());
  Serial.println("> connected.\n");
  
  // initialize SNTP
  esp_sntp_init();
  esp_sntp_setservername(sntpServerID, sntpTimeServer1);
  configTime(gmtOffset, daylightOffset, sntpTimeServer1, sntpTimeServer2);
  configTzTime(timeZone, sntpTimeServer1, sntpTimeServer2);
  Serial.println("SNTP initialized...\n");
  
  // initialize over the air serial monitor
  #ifdef OTA_OUTPUT
    WebSerial.begin(&serialOTA);
    WebSerial.onMessage(serialOTAReceiver);
    serialOTA.begin();
    WebSerial.setID("Smart Meter DTSU666 Prod Home Assistant Integration");
  #endif
  
  // initialize MQTT server connection
  Serial.print("MQTT server <");
  Serial.print(mqttServer);
  Serial.print("> connecting...");
  mqttClient.begin(mqttServer, mqttServerPort, wifiConnectionID);
  delay(CONNECTION_DELAY);
  mqttClient.onMessage(mqttMessageReceiver);
  mqttReconnectionCounter = 0;
  while(!mqttClient.connected() && (mqttReconnectionCounter < mqttMaxReconnections)) {
    mqttReconnectionCounter++;
    mqttClient.connect(mqttClientID, mqttUser, mqttPassword, mqttDisconnectSkip);
    Serial.print("-");
    delay(CONNECTION_RETRY_DELAY);
  }
  if(mqttReconnectionCounter >= mqttMaxReconnections) {
    sprintf(msgString, "\nMQTT Error: MQTT server not connected after <%u> retries", mqttReconnectionCounter);
    Serial.println(msgString);
    
    esp32Restart(ESP32_RESTART_MQTT_CONNECTION);
  }
  Serial.println("\nMQTT server connected.");
  Serial.println();
  
  if((WiFi.status() == WL_CONNECTED) && mqttClient.connected()) {
    connectionsEstablished = 1;
    
    // output converter states
    mqttOutputConverterState();
    
    // MQTT subscriptions
    #ifdef DEBUG_OUTPUT
      Serial.println("Smart Meter DTSU666 Prod [MQTT Subscriptions]");
    #endif
    
    // MQTT keep alive subscription
    mqttTopicID = "MQTT";
    mqttTopicEntity = "Keep_Alive_DTSU666";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    delay(MQTT_SUBSCRIBE_DELAY);
    #ifdef DEBUG_OUTPUT
      Serial.println(mqttTopicID);
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
  }
  
  // reset counters
  mqttReconnectionCounter = 0;
  modbusErrorCounter = 0;
  dtsu666RegisterBlockIndex = 0;
  
  // start chronos
  mqttKeepAliveChrono.start();
  dtsu666DataHandleChrono.start();
  refreshOutputStateChrono.start();
  
  #ifdef DEBUG_OUTPUT
    Serial.println();
  #endif
  
  // output converter + dtsu666 states
  mqttOutputConverterState();
  outputInfoline();
}  // end setup()

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  // handle ESP32 Halt
  if(esp32Halt) {
    esp32Halt = false;
    
    Serial.println("ESP32 Halt...");
    #ifdef OTA_OUTPUT
      WebSerial.println("ESP32 Halt...");
    #endif
    
    delay(ESP32_HALT_DELAY);
    esp32Restart(ESP32_RESTART_OTA_HALT);
  }
  
  #ifdef ESP32_RESTART_TIME
  // restart only if ESP32_RESTART_TIME is defined
    if(actTimestamp == ESP32_RESTART_TIME) {
      esp32Restart(ESP32_RESTART_OTA_HALT);
    }
  #endif
  
  // handle SNTP
  if(getLocalTime(&timeInfo)) {
    char timeInfoDate[16];
    strftime(timeInfoDate, 16, "%F", &timeInfo);
    char timeInfoTime[16];
    strftime(timeInfoTime, 16, "%T", &timeInfo);
    char timeInfoHourMinute[8];
    strftime(timeInfoHourMinute, 8, "%R", &timeInfo);
    
    sprintf(msgString, "%s / %s", timeInfoDate, timeInfoTime);  // "yyyy-mm-dd / hh:mm:ss"
    actDateTime = String(msgString).c_str();
    sprintf(msgString, "%s", timeInfoHourMinute);  // "hh:mm"
    actTimestamp = String(msgString).c_str();
  }
  
  // handle WebSerial loop
  #ifdef OTA_OUTPUT
    WebSerial.loop();
  #endif
  
  // handle MQTT loop
  mqttClient.loop();
  delay(MQTT_LOOP_DELAY);
  
  // output converter + dtsu666 states
  if(refreshOutputStateChrono.hasPassed(STATE_OUTPUT_REFRESH_INTERVAL_CHRONO)) {
    mqttOutputConverterState();
    outputInfoline();
    refreshOutputStateChrono.restart();
  }
  
  // handle MQTT keep alive
  if(mqttKeepAliveChrono.hasPassed(MQTT_KEEP_ALIVE_INTERVAL_CHRONO) && connectionsEstablished) {
    mqttKeepAliveCounter++;
    
    mqttTopicID = "MQTT";
    mqttTopicEntity = "Keep_Alive_DTSU666";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttKeepAliveCounter) + "\":\"" + String(mqttKeepAliveReceivedCounter) + "\":\"" + String(mqttKeepAliveDelayedCounter) + "\"}").c_str();
    mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
    
    mqttKeepAliveChrono.restart();
    if(! mqttKeepAliveDelayChrono.isRunning()) {
      mqttKeepAliveDelayChrono.start();
    } else {
      mqttKeepAliveDelayChrono.restart();
    }
  }  // end if mqttKeepAliveChrono.hasPassed
  
  // handle MQTT keep alive delayes
  if(mqttKeepAliveDelayChrono.hasPassed(MQTT_KEEP_ALIVE_DELAY_CHRONO) && (mqttKeepAliveCounter != 0)) {
    mqttKeepAliveDelayedCounter++;
    
    sprintf(msgString, "MQTT Connection delayed %lux!\n", mqttKeepAliveDelayedCounter);
    Serial.println(msgString);
    #ifdef OTA_OUTPUT
      sprintf(msgString, "MQTT Connection delayed %lux!\n", mqttKeepAliveDelayedCounter);
      WebSerial.println(msgString);
    #endif
    
    if(! mqttKeepAliveDelayChrono.isRunning()) {
      mqttKeepAliveDelayChrono.start();
    } else {
      mqttKeepAliveDelayChrono.restart();
    }
    
    // ESP32 restart at <mqttMaxReconnections> keep alive delayes
    if((mqttKeepAliveDelayedCounter % MQTT_KEEP_ALIVE_MAX_DELAYS) == 0) {
      esp32Restart(ESP32_RESTART_MODBUS_KEEP_ALIVE);
    }
  }  // end if mqttKeepAliveDelayChrono.hasPassed
  
  // handle dtsu666 data register blocks
  if(dtsu666DataHandleChrono.hasPassed(DTSU666_DATA_HANDLE_INTERVAL_CHRONO)) {
    mqttTopicID = "DTSU666";
    
    switch(dtsu666RegisterBlockIndex) {
      case 0: {  // System
        dtsu666RegisterBlockName = "System";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterSystem[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterSystem[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterSystem[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterSystem[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterSystem[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterSystem[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterSystem[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 1: {  // Data0
        dtsu666RegisterBlockName = "Data0";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData0[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData0[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterData0[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData0[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData0[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData0[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData0[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 2: {  // Data1
        dtsu666RegisterBlockName = "Data1";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData1[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData1[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterData1[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData1[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData1[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData1[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData1[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 3: {  // Data2
        dtsu666RegisterBlockName = "Data2";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData2[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData2[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterData2[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData2[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData2[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData2[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData2[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 4: {  // Data3
        dtsu666RegisterBlockName = "Data3";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData3[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData3[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterData3[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData3[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData3[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData3[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData3[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 5: {  // Data4
        dtsu666RegisterBlockName = "Data4";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s/%s", String(mqttTopicID).c_str(), String(dtsu666RegisterBlockName).c_str());
              WebSerial.println(msgString);
            #endif
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData4[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData4[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDescriptor = dtsu666RegisterData4[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData4[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData4[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData4[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData4[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDescriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            handleModbusErrorCounters(resultModbus);
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [%s (0x%0.2x)]: modbusNode.readHoldingRegisters(0x%0.4x, %u)", modbusReturnCodes[(resultModbus - MODBUS_ERROR_CODE_OFFSET)], resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.println();
          #endif
          #ifdef OTA_OUTPUT
            WebSerial.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex = 0;  // reset dtsu666 register block index
        break;
      }  // end case
      default: {
        dtsu666RegisterBlockIndex = 0;  // reset dtsu666 register block index
        break;
      }  // end default
    }  // end switch dtsu666RegisterBlockIndex
  }  // end if dtsu666DataHandleChrono.hasPassed
}  // end loop()

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void esp32Restart(uint8_t esp32RestartID) {
  String mqttTopicID = "ESP32";
  String mqttTopicEntity = "Restart";
  String mqttPayload="1";
  bool mqttSuccess;
  
  // adjust+store NVS ESP32 restart counters
  esp32RestartInfo[esp32RestartID].esp32RestartCounter++;
  nvs.putUInt(esp32RestartInfo[esp32RestartID].esp32RestartReasonNVS, esp32RestartInfo[esp32RestartID].esp32RestartCounter);
  
  Serial.println("ESP32 Restarting...");
  #ifdef OTA_OUTPUT
    WebSerial.println("ESP32 Restarting...");
  #endif

  // MQTT publish
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayload) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  
  // reset connections
  RS485.end();
  delay(CONNECTION_DELAY);
  mqttClient.disconnect();
  delay(CONNECTION_DELAY);
  esp_sntp_stop();
  delay(CONNECTION_DELAY);
  WiFi.disconnect();
  
  // NVS end
  nvs.end();
  
  delay(ESP32_RESTART_DELAY);
  ESP.restart();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void serialOTAReceiver(uint8_t *data, size_t length) {
  String buffer = "";
  
  for(uint8_t i = 0; i < length; i++) {
    buffer += char(data[i]);
  }
  
  if(buffer == "Restart") {
    esp32Restart(ESP32_RESTART_OTA_RESTART);
  } else if(buffer == "Halt") {
    esp32Halt = 1;
  } else if(buffer == "Clear") {
    // clear NVS
    nvs.clear();
    
    // read NVS values
    for(uint8_t i = 0; i < ESP32_RESTART_INFO_ELEMENTS; i++) {
      esp32RestartInfo[i].esp32RestartCounter = nvs.getUInt(esp32RestartInfo[i].esp32RestartReasonNVS);
    }
    
    WebSerial.println("ESP32: NVS cleared.");
  } else {
    WebSerial.println("Usage: \'Restart\' | \'Halt\' | \'Clear\'");
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void handleModbusErrorCounters(uint8_t resultModbus) {
  switch(resultModbus) {
    case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_INVALID_SLAVE_ID]++; break;
    case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_INVALID_FUNCTION]++; break;
    case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_RESPONCE_TIMEOUT]++; break;
    case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_INVALID_CRC]++; break;
    default: break;
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttOutputConverterState() {
  for(unsigned short element = 0; element < DTSU666_MODBUS_CONVERTER_MQTT_OUTPUT_ELEMENTS; element++) {
    switch(element) {
      case 0: {  // DTSU666_Modbus_Converter_Error_0xE0
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[0];
        break;
      }  // end case
      case 1: {  // DTSU666_Modbus_Converter_Error_0xE1
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[1];
        break;
      }  // end case
      case 2: {  // DTSU666_Modbus_Converter_Error_0xE2
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[2];
        break;
      }  // end case
      case 3: {  // DTSU666_Modbus_Converter_Error_0xE3
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[3];
        break;
      }  // end case
      case 4: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveCounter;
        break;
      }  // end case
      case 5: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive_Received
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveReceivedCounter;
        break;
      }  // end case
      case 6: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive_Delayed
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTOutputArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveDelayedCounter;
        break;
      }  // end case
      default: {
        break;
      }  // end default
    }  // end switch element

    mqttTopicID = "DTSU666";
    mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, mqttTopicEntity, mqttPayloadData, mqttRetained, mqttQoS);
  }  // end for element
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputInfoline() {
  #ifdef DEBUG_OUTPUT
    sprintf(msgString, "Smart Meter DTSU666 Prod - %s", String(actDateTime).c_str());
    Serial.println(msgString);
    
    sprintf(msgString, "ESP32 Restart Counters: [%lu | %lu | %lu | %lu | %lu | %lu] - [%s | %s | %s | %s | %s | %s]", \
                                                 esp32RestartInfo[0].esp32RestartCounter, esp32RestartInfo[1].esp32RestartCounter, esp32RestartInfo[2].esp32RestartCounter, esp32RestartInfo[3].esp32RestartCounter, esp32RestartInfo[4].esp32RestartCounter, esp32RestartInfo[5].esp32RestartCounter, \
                                                 esp32RestartInfo[0].esp32RestartReasonNVS, esp32RestartInfo[1].esp32RestartReasonNVS, esp32RestartInfo[2].esp32RestartReasonNVS, esp32RestartInfo[3].esp32RestartReasonNVS, esp32RestartInfo[4].esp32RestartReasonNVS, esp32RestartInfo[5].esp32RestartReasonNVS);
    Serial.println(msgString);
    
    sprintf(msgString, "Modbus Errors [%lu]: [%lu | %lu | %lu | %lu] - [InvalidSlaveID | InvalidFunction | ResponseTimedOut | InvalidCRC]", modbusErrorCounter, modbusErrorCodes[1], modbusErrorCodes[2], modbusErrorCodes[3], modbusErrorCodes[4]);
    Serial.println(msgString);
    
    sprintf(msgString, "MQTT Counters: [%lu | %lu | %lu] - [KeepAlive | Received | Delayed]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    Serial.println(msgString);
    sprintf(msgString, "MQTT Chronos: [%02lus:%02lus | %02lus:%02lus] - [KeepAliveCrono | KeepAliveDelayChrono]", MQTT_KEEP_ALIVE_INTERVAL_CHRONO, mqttKeepAliveChrono.elapsed(), MQTT_KEEP_ALIVE_DELAY_CHRONO, mqttKeepAliveDelayChrono.elapsed());
    Serial.println(msgString);
    
    Serial.println();
  #endif
  
  #ifdef OTA_OUTPUT
    sprintf(msgString, "Smart Meter DTSU666 Prod - %s", String(actDateTime).c_str());
    WebSerial.println(msgString);
    
    sprintf(msgString, "ESP32 Restart Counters: [%lu | %lu | %lu | %lu | %lu | %lu] - [%s | %s | %s | %s | %s | %s]", \
                                                 esp32RestartInfo[0].esp32RestartCounter, esp32RestartInfo[1].esp32RestartCounter, esp32RestartInfo[2].esp32RestartCounter, esp32RestartInfo[3].esp32RestartCounter, esp32RestartInfo[4].esp32RestartCounter, esp32RestartInfo[5].esp32RestartCounter, \
                                                 esp32RestartInfo[0].esp32RestartReasonNVS, esp32RestartInfo[1].esp32RestartReasonNVS, esp32RestartInfo[2].esp32RestartReasonNVS, esp32RestartInfo[3].esp32RestartReasonNVS, esp32RestartInfo[4].esp32RestartReasonNVS, esp32RestartInfo[5].esp32RestartReasonNVS);
    WebSerial.println(msgString);
    
    sprintf(msgString, "Modbus Errors [%lu]: [%lu | %lu | %lu | %lu] - [InvalidSlaveID | InvalidFunction | ResponseTimedOut | InvalidCRC]", modbusErrorCounter, modbusErrorCodes[1], modbusErrorCodes[2], modbusErrorCodes[3], modbusErrorCodes[4]);
    WebSerial.println(msgString);
    
    sprintf(msgString, "MQTT Counters: [%lu | %lu | %lu] - [KeepAlive | Received | Delayed]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    WebSerial.println(msgString);
    sprintf(msgString, "MQTT Chronos: [%02lus:%02lus | %02lus:%02lus] - [KeepAliveCrono | KeepAliveDelayChrono]", MQTT_KEEP_ALIVE_INTERVAL_CHRONO, mqttKeepAliveChrono.elapsed(), MQTT_KEEP_ALIVE_DELAY_CHRONO, mqttKeepAliveDelayChrono.elapsed());
    WebSerial.println(msgString);
    
    WebSerial.println();
  #endif
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttMessageReceiver(String &inputTopic, String &inputPayload) {
  String mqttTopic;
  String mqttTopicID;
  String mqttTopicEntity;
  
  // [MQTT/Keep_Alive_DTSU666]
  mqttTopicID = "MQTT";
  mqttTopicEntity = "Keep_Alive_DTSU666";
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  if(inputTopic == mqttTopic) {
    mqttKeepAliveReceivedCounter++;
    mqttKeepAliveDelayChrono.stop();
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "MQTT Message Receiver: %s - %s", String(inputTopic).c_str(), String(inputPayload).c_str());
      Serial.println(msgString);
      Serial.println();
    #endif
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputDTSU666Data(uint16_t dtsu666EntityResponseBufferOffset, char *dtsu666EntityDescriptor, char *dtsu666EntityDataType, float dtsu666EntityDataFactor, char *dtsu666EntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopicID = "DTSU666";
  bool mqttSuccess;
  
  if(dtsu666EntityDataType == "CHAR") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    bufferValueChar = (bufferValueUInt16 & 0x00FF);  // 8 bit conversion
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %c", String(dtsu666EntityDescriptor).c_str(), bufferValueChar);
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %c", String(dtsu666EntityDescriptor).c_str(), bufferValueChar);
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      //mqttSuccess = mqttPublishTopicPayloadChar(mqttTopicID, dtsu666EntityDescriptor, bufferValueChar, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "I16") {
    bufferValueInt16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %d%s", String(dtsu666EntityDescriptor).c_str(), bufferValueInt16, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %d%s", String(dtsu666EntityDescriptor).c_str(), bufferValueInt16, String(dtsu666EntityUnit).c_str());
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadInt16(mqttTopicID, dtsu666EntityDescriptor, bufferValueInt16, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "U16") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDescriptor).c_str(), bufferValueUInt16, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDescriptor).c_str(), bufferValueUInt16, String(dtsu666EntityUnit).c_str());
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt16(mqttTopicID, dtsu666EntityDescriptor, bufferValueUInt16, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "U32") {
    bufferValueUInt32 = ((modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset) << 16) + modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset + 1));
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDescriptor).c_str(), bufferValueUInt32, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDescriptor).c_str(), bufferValueUInt32, String(dtsu666EntityUnit).c_str());
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, dtsu666EntityDescriptor, bufferValueUInt32, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "F16") {
    union {
      float float16;
      uint16_t uint16;
    } conversion;
    float bufferValueFloat16;

    conversion.uint16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    bufferValueFloat16 = (conversion.float16 * dtsu666EntityDataFactor);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDescriptor).c_str(), bufferValueFloat16, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDescriptor).c_str(), bufferValueFloat16, String(dtsu666EntityUnit).c_str());
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat16(mqttTopicID, dtsu666EntityDescriptor, bufferValueFloat16, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "F32") {
    union {
      float float32;
      uint32_t uint32;
    } conversion;
    float bufferValueFloat32;

    conversion.uint32 = ((modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset) << 16) + modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset + 1));
    bufferValueFloat32 = (conversion.float32 * dtsu666EntityDataFactor);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDescriptor).c_str(), bufferValueFloat32, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDescriptor).c_str(), bufferValueFloat32, String(dtsu666EntityUnit).c_str());
      WebSerial.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat32(mqttTopicID, dtsu666EntityDescriptor, bufferValueFloat32, mqttRetained, mqttQoS);
    }
  } else {
    #ifdef DEBUG_OUTPUT
      Serial.print("Unknown Type: ");
      Serial.println(dtsu666EntityDataType);
    #endif
    #ifdef OTA_OUTPUT
      WebSerial.print("Unknown Type: ");
      WebSerial.println(dtsu666EntityDataType);
    #endif
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
bool mqttPublishTopicPayloadCHAR(String mqttTopicID, String mqttTopicEntity, char mqttPayloadCHAR, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadCHAR) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadInt16(String mqttTopicID, String mqttTopicEntity, int16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadUInt16) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadUInt16(String mqttTopicID, String mqttTopicEntity, uint16_t mqttPayloadUInt16, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadUInt16) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadUInt32(String mqttTopicID, String mqttTopicEntity, uint32_t mqttPayloadUInt32, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadUInt32) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadFloat16(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat16, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;
  
  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadFloat16, 2) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

bool mqttPublishTopicPayloadFloat32(String mqttTopicID, String mqttTopicEntity, float mqttPayloadFloat32, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopic;
  String mqttPayload;
  bool mqttSuccess;

  mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  mqttPayload = ("{\"" + String(mqttTopicEntity) + "\":\"" + String(mqttPayloadFloat32, 2) + "\"}").c_str();
  mqttSuccess = mqttClient.publish(mqttTopic, mqttPayload, mqttRetained, mqttQoS);
  delay(MQTT_PUBLISH_DELAY);
  return(mqttSuccess);
}

//EOF