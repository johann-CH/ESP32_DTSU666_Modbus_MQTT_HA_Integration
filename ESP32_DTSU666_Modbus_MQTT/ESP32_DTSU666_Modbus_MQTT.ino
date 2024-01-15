/*
  ESP32 + RS485 Converter: Smart Meter DTSU666 Prod Home Assistant Integration (RS485(Modbus)/MQTT)
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <ESP32.h>

#include <Chrono.h>
#include <WiFi.h>
#include <NTP.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialPro.h>

#include <ModbusMaster.h>
#include <MQTT.h>

#include "ESP32_DTSU666_Modbus_MQTT.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
#define SERIAL_BAUDRATE 115200  // default serial interface bauderate
#define SERIAL_DELAY 200        // serial communication delay [ms]

#define RS485 Serial2              // rename serial interface name
#define RS485_BAUDRATE 9600        // serial interface bauderate
#define RS485_MODE SERIAL_8N1      // serial mode (1 start / 8 data / 1 stop / no parity)
#define RS485_IO_RX 16             // GPIO16:RX2 (ESP32)
#define RS485_IO_TX 17             // GPIO17:TX2 (ESP32)
#define RS485_DELAY 200            // RS485 communication delay [ms]
#define RS485_RX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]
#define RS485_TX_BUFFER_SIZE 2048  // RS485 RX buffer size [byte]

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

#define DTSU666_DATA_HANDLE_INTERVAL_CHRONO 2  // chrono dtsu666 register data handle interval [s]

#define STATE_OUTPUT_REFRESH_INTERVAL_CHRONO 10  // chrono dtsu666 + converter states output refresh interval [s]

#define DEBUG_OUTPUT  // default serial interface output active

#define OTA_OUTPUT  // over the air serial monitor output active

#define ESP32_RESTART_TIMESTAMP "06:00"  // ESP32 restart timestamp 06h00
#define ESP32_RESTART_LOCK_DELAY 60000   // ESP32 restart lock delay 60s [ms]
#define ESP32_RESTART_DELAY 5000         // ESP32 restart command delay [ms]

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// instantiate chrono objects
Chrono mqttKeepAliveChrono(Chrono::SECONDS);
Chrono mqttKeepAliveDelayChrono(Chrono::SECONDS);
Chrono dtsu666DataHandleChrono(Chrono::SECONDS);
Chrono refreshOutputStateChrono(Chrono::SECONDS);

// instantiate WiFi+NTP+MQTT objects
WiFiClient wifiConnectionID;
WiFiUDP wifiUDP;
NTP ntp(wifiUDP);
MQTTClient mqttClient(MQTT_IO_BUFFER_SIZE);

// instantiate Modbus object
ModbusMaster modbusNode;
/* ModbudMaster Error Codes
  ku8MBSuccess          = 0x00;
  ku8MBInvalidSlaveID   = 0xE0;
  ku8MBInvalidFunction  = 0xE1;
  ku8MBResponseTimedOut = 0xE2;
  ku8MBInvalidCRC       = 0xE3;
 */

// instantiate over the air serial monitor
#ifdef OTA_OUTPUT
  AsyncWebServer serialOTA(80);
#endif

// WiFi definitions
const char wifiSSID[] = "WLAN_Belp";  // WiFi network SSID
const char wifiPassword[] = "family.ruch@belp";  // WiFi network password

unsigned int wifiReconnectionCounter = 0;
const unsigned int wifiMaxReconnections = 10;

// NTP definitions
String actDateTime = "1970-01-01 / 00:00:00";
String actTimestamp = "00:00";

// Modbus definitions
unsigned int resultModbus;
unsigned long modbusErrorCounter = 0;
unsigned long modbusErrorCodes[4];

// MQTT definitions
const char mqttServer[] = "192.168.1.150";
int mqttServerPort = 1883;
const char mqttUser[] = "mqtt-admin";
const char mqttPassword[] = "mqtt-admin";
const char mqttClientID[] = "dtsu666_mqtt";
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
char *dtsu666EntityDesriptor;
char *dtsu666EntityDataType;
float dtsu666EntityDataFactor;
char *dtsu666EntityUnit;
bool dtsu666EntityActiveMQTT;
char bufferValueChar;
uint16_t bufferValueUInt16;
uint32_t bufferValueUInt32;

bool mqttSuccess;

unsigned int mqttReconnectionCounter = 0;
const unsigned int mqttMaxReconnections = 10;

unsigned long mqttKeepAliveCounter = 0;
unsigned long mqttKeepAliveReceivedCounter = 0;
unsigned long mqttKeepAliveDelayedCounter = 0;

// common definitions
char msgString[256];  // message char array

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
  Serial.begin(SERIAL_BAUDRATE);
  Serial.flush();
  delay(SERIAL_DELAY);
  
  // initialize serial interface (RS485)
  RS485.begin(RS485_BAUDRATE, RS485_MODE, RS485_IO_RX, RS485_IO_TX);
  RS485.setRxBufferSize(RS485_RX_BUFFER_SIZE);
  RS485.setTxBufferSize(RS485_TX_BUFFER_SIZE);
  RS485.flush();
  delay(RS485_DELAY);
  
  // output infoline
  Serial.println("Smart Meter DTSU666 Prod [Init Modbus + WiFi + MQTT]");
  Serial.print("> / <");
  
  // initalize modbus slave <ID>
  modbusNode.begin(MODBUS_SLAVE_ID, RS485);
  delay(MODBUS_DELAY);
  modbusNode.clearResponseBuffer();
  
  // initialize WiFi network connection
  Serial.print("WiFi connecting...");
  WiFi.mode(WIFI_STA);  // WiFi station mode
  WiFi.begin(wifiSSID, wifiPassword);
  delay(CONNECTION_DELAY);
  wifiReconnectionCounter = 0;
  while((WiFi.status() != WL_CONNECTED) && (wifiReconnectionCounter < wifiMaxReconnections)) {
    wifiReconnectionCounter++;
    WiFi.reconnect();
    Serial.print("-");
    delay(CONNECTION_RETRY_DELAY);
  }
  if(wifiReconnectionCounter >= wifiMaxReconnections) {
    WiFi.disconnect();
    sprintf(msgString, "\nWiFi Error: WiFi not connected after <%u> retries", wifiReconnectionCounter);
    Serial.println(msgString);
    
    esp32Restart();
  }
  Serial.print("\nWiFi <");
  Serial.print(wifiSSID);
  Serial.print("> / <");
  Serial.println(WiFi.localIP());
  Serial.println("> connected.\n");
  
  // initialize NTP
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120);  // last sunday in march 2h00, timezone +120min (+1 GMT / +1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60);    // last sunday in october 3h00, timezone +60min (+1 GMT)
  ntp.begin();
  Serial.println("NTP initialized...\n");
  
  // initialize over the air serial monitor
  #ifdef OTA_OUTPUT
    WebSerialPro.begin(&serialOTA);
    WebSerialPro.msgCallback(serialOTAReceiver);
    serialOTA.begin();
    WebSerialPro.setID("Smart Meter DTSU666 Prod Home Assistant Integration");
  #endif
  
  // initialize MQTT server connection
  Serial.print("MQTT server <");
  Serial.print(mqttServer);
  Serial.print("> connecting...");
  mqttClient.begin(mqttServer, mqttServerPort, wifiConnectionID);
  delay(CONNECTION_DELAY);
  mqttClient.onMessage(mqttKeepAliveReceiver);
  mqttReconnectionCounter = 0;
  while(!mqttClient.connected() && (mqttReconnectionCounter < mqttMaxReconnections)) {
    mqttReconnectionCounter++;
    mqttClient.connect(mqttClientID, mqttUser, mqttPassword, mqttDisconnectSkip);
    Serial.print("-");
    delay(CONNECTION_RETRY_DELAY);
  }
  if(mqttReconnectionCounter >= mqttMaxReconnections) {
    mqttClient.disconnect();
    sprintf(msgString, "\nMQTT Error: MQTT server not connected after <%u> retries", mqttReconnectionCounter);
    Serial.println(msgString);
    
    esp32Restart();
  }
  Serial.println("\nMQTT server connected.");
  Serial.println();
  
  if((WiFi.status() == WL_CONNECTED) && mqttClient.connected()) {
    connectionsEstablished = 1;
    
    // MQTT output converter states
    mqttOutputConverterState();
    
    // MQTT subscriptions
    #ifdef DEBUG_OUTPUT
      Serial.println("Smart Meter DTSU666 Prod [MQTT Subscriptions]");
    #endif
    
    // MQTT ESP32 subscription
    mqttTopicID = "ESP32";
    mqttTopicEntity = "Restart";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    #ifdef DEBUG_OUTPUT
      Serial.println("ESP32");
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
    delay(MQTT_SUBSCRIBE_DELAY);
    
    // MQTT dtsu666 modbus converter subscriptions
    #ifdef DEBUG_OUTPUT
      Serial.println("DTSU666ModbusConverterStates");
    #endif
    for(unsigned short element = 0; element < DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS; element++) {
      mqttTopicID = "DTSU666Converter";
      mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
      mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
      mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
      #ifdef DEBUG_OUTPUT
        sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
        Serial.println(msgString);
      #endif
      delay(MQTT_SUBSCRIBE_DELAY);
    }
    
    // MQTT keep alive subscription
    mqttTopicID = "MQTT";
    mqttTopicEntity = "Keep_Alive_DTSU666";
    mqttTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
    mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
    #ifdef DEBUG_OUTPUT
      Serial.println("MQTT");
      sprintf(msgString, "%s", String(mqttTopicID + "/" + mqttTopicEntity).c_str());
      Serial.println(msgString);
    #endif
    delay(MQTT_SUBSCRIBE_DELAY);
    
    // MQTT dtsu666 register block subscriptions
    for(dtsu666RegisterBlockIndex = 0; dtsu666RegisterBlockIndex < DTSU666_REGISTER_BLOCK_COUNT; dtsu666RegisterBlockIndex++) {
      switch(dtsu666RegisterBlockIndex) {
        case 0: {  // Data0
          dtsu666RegisterBlockName = "Data0";
          if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666EntityDesriptor = dtsu666RegisterData0[element].dtsu666RegisterDescriptor;
              dtsu666EntityActiveMQTT = dtsu666RegisterData0[element].dtsu666RegisterActiveMQTT;
              
              if(dtsu666EntityActiveMQTT) {
                mqttTopicID = "DTSU666";
                mqttTopic = String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str(), dtsu666RegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if dtsu666RegisterBlockActive
          break;
        }  // end case
        case 1: {  // Data1
          dtsu666RegisterBlockName = "Data1";
          if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666EntityDesriptor = dtsu666RegisterData1[element].dtsu666RegisterDescriptor;
              dtsu666EntityActiveMQTT = dtsu666RegisterData1[element].dtsu666RegisterActiveMQTT;
              
              if(dtsu666EntityActiveMQTT) {
                mqttTopicID = "DTSU666";
                mqttTopic = String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str(), dtsu666RegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if dtsu666RegisterBlockActive
          break;
        }  // end case
        case 2: {  // Data2
          dtsu666RegisterBlockName = "Data2";
          if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666EntityDesriptor = dtsu666RegisterData2[element].dtsu666RegisterDescriptor;
              dtsu666EntityActiveMQTT = dtsu666RegisterData2[element].dtsu666RegisterActiveMQTT;
              
              if(dtsu666EntityActiveMQTT) {
                mqttTopicID = "DTSU666";
                mqttTopic = String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str(), dtsu666RegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if dtsu666RegisterBlockActive
          break;
        }  // end case
        case 3: {  // Data3
          dtsu666RegisterBlockName = "Data3";
          if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666EntityDesriptor = dtsu666RegisterData3[element].dtsu666RegisterDescriptor;
              dtsu666EntityActiveMQTT = dtsu666RegisterData3[element].dtsu666RegisterActiveMQTT;
              
              if(dtsu666EntityActiveMQTT) {
                mqttTopicID = "DTSU666";
                mqttTopic = String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str(), dtsu666RegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if dtsu666RegisterBlockActive
          break;
        }  // end case
        case 4: {  // Data4
          dtsu666RegisterBlockName = "Data4";
          if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666EntityDesriptor = dtsu666RegisterData4[element].dtsu666RegisterDescriptor;
              dtsu666EntityActiveMQTT = dtsu666RegisterData4[element].dtsu666RegisterActiveMQTT;
              
              if(dtsu666EntityActiveMQTT) {
                mqttTopicID = "DTSU666";
                mqttTopic = String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str();
                mqttSuccess = mqttClient.subscribe(mqttTopic, mqttQoS);
                #ifdef DEBUG_OUTPUT
                  sprintf(msgString, "%s [%u/%u]", String(mqttTopicID + "/" + dtsu666EntityDesriptor).c_str(), dtsu666RegisterBlockIndex, element);
                  Serial.println(msgString);
                #endif
                delay(MQTT_SUBSCRIBE_DELAY);
              }
            }
          }  // end if dtsu666RegisterBlockActive
          break;
        }  // end case
        default: {
          dtsu666RegisterBlockIndex = 0;  // reset dtsu666 register block index
          break;
        }  // end default
      }  // end switch dtsu666RegisterBlockIndex
    }  // end for dtsu666RegisterBlockIndex
    
   dtsu666RegisterBlockIndex = 0;  // reset dtsu666 register block index
  }
  
  // reset reconnection counters
  wifiReconnectionCounter = 0;
  mqttReconnectionCounter = 0;
  modbusErrorCounter = 0;
  
  // start chronos
  mqttKeepAliveChrono.start();
  dtsu666DataHandleChrono.start();
  refreshOutputStateChrono.start();
  
  #ifdef DEBUG_OUTPUT
    Serial.print("> / <");
  #endif
  
  // output dtsu666 + converter states 
  mqttOutputConverterState();
  outputInfoline();
}  // end setup()

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
   #ifdef LED_ACTIVE
    if(LEDFlashChrono.hasPassed(LEDFlashInterval)) {
      // flash LED to indicate activity
      stateLED = !(stateLED);
      digitalWrite(LED_BUILTIN, stateLED);
      LEDFlashChrono.restart();
    }
  #endif

  // handle NTP
  ntp.update();
  actDateTime = ntp.formattedTime("%Y-%m-%d / %H:%M:%S");
  actTimestamp = ntp.formattedTime("%H:%M");
  
  #ifdef ESP32_RESTART_TIMESTAMP
  // restart only if ESP32_RESTART_TIMESTAMP is defined
    if(actTimestamp == ESP32_RESTART_TIMESTAMP) {
      esp32Restart();
    }
  #endif
  
  // handle MQTT loop
  mqttClient.loop();
  delay(MQTT_LOOP_DELAY);
  
  // output dtsu666 + converter states
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
      WebSerialPro.println(msgString);
    #endif
    
    // ESP32 restart at <mqttMaxReconnections> keep alive delayes
    if((mqttKeepAliveDelayedCounter % 10) == 0) {
      esp32Restart();
    }
  }  // end if mqttKeepAliveDelayChrono.hasPassed

  if(dtsu666DataHandleChrono.hasPassed(DTSU666_DATA_HANDLE_INTERVAL_CHRONO)) {
    // handle dtsu666 data
    switch(dtsu666RegisterBlockIndex) {
      case 0: {  // Data0
        dtsu666RegisterBlockName = "Data0";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "DTSU666";
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData0[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData0[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDesriptor = dtsu666RegisterData0[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData0[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData0[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData0[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData0[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDesriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.print("> / <");
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 1: {  // Data1
        dtsu666RegisterBlockName = "Data1";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "DTSU666";
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData1[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData1[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDesriptor = dtsu666RegisterData1[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData1[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData1[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData1[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData1[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDesriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.print("> / <");
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 2: {  // Data2
        dtsu666RegisterBlockName = "Data2";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "DTSU666";
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData2[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData2[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDesriptor = dtsu666RegisterData2[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData2[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData2[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData2[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData2[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDesriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.print("> / <");
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 3: {  // Data3
        dtsu666RegisterBlockName = "Data3";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "DTSU666";
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData3[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData3[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDesriptor = dtsu666RegisterData3[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData3[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData3[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData3[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData3[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDesriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.print("> / <");
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
        break;
      }  // end case
      case 4: {  // Data4
        dtsu666RegisterBlockName = "Data4";
        if(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockActive) {
          resultModbus = modbusNode.readHoldingRegisters(dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
          delay(MODBUS_DELAY);
          RS485.flush();
          if(resultModbus == modbusNode.ku8MBSuccess) {
            dtsu666RegisterBlockHead = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead;
            dtsu666RegisterBlockSize = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize;
            dtsu666RegisterBlockElements = dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockElements;
            
            // handle modbus holding register data
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              Serial.println(msgString);
            #endif
            #ifdef OTA_OUTPUT
              sprintf(msgString, "%s", String(dtsu666RegisterBlockName).c_str());
              WebSerialPro.println(msgString);
            #endif
            
            mqttTopicID = "DTSU666";
            
            for(unsigned short element = 2; element < dtsu666RegisterBlockElements; element++) {
              dtsu666RegisterBlockBaseAddress = dtsu666RegisterData4[0].dtsu666RegisterAddress;
              dtsu666EntityRegisterAddress = dtsu666RegisterData4[element].dtsu666RegisterAddress;
              dtsu666EntityResponseBufferOffset = (dtsu666EntityRegisterAddress - dtsu666RegisterBlockBaseAddress);
              dtsu666EntityDesriptor = dtsu666RegisterData4[element].dtsu666RegisterDescriptor;
              dtsu666EntityDataType = dtsu666RegisterData4[element].dtsu666RegisterDataType;
              dtsu666EntityDataFactor = dtsu666RegisterData4[element].dtsu666RegisterDataFactor;
              dtsu666EntityUnit = dtsu666RegisterData4[element].dtsu666RegisterUnit;
              dtsu666EntityActiveMQTT = dtsu666RegisterData4[element].dtsu666RegisterActiveMQTT;
              
              outputDTSU666Data(dtsu666EntityResponseBufferOffset, dtsu666EntityDesriptor, dtsu666EntityDataType, dtsu666EntityDataFactor, dtsu666EntityUnit, dtsu666EntityActiveMQTT, mqttRetained, mqttQoS);
            }
          } else {
            modbusErrorCounter++;
            switch(resultModbus) {
              case modbusNode.ku8MBInvalidSlaveID: modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX]++; break;
              case modbusNode.ku8MBInvalidFunction: modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX]++; break;
              case modbusNode.ku8MBResponseTimedOut: modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX]++; break;
              case modbusNode.ku8MBInvalidCRC: modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]++; break;
            }
            #ifdef DEBUG_OUTPUT
              sprintf(msgString, "Modbus Error [0x%0.2x]: modbusNode.readHoldingRegisters(0x%0.2x, %u)", resultModbus, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockHead, dtsu666RegisterBlocks[dtsu666RegisterBlockIndex].dtsu666RegisterBlockSize);
              Serial.println(msgString);
            #endif
          }  // end if resultModbus
          modbusNode.clearResponseBuffer();
          dtsu666DataHandleChrono.restart();  // restart chrono

          #ifdef DEBUG_OUTPUT
            Serial.print("> / <");
          #endif
          #ifdef OTA_OUTPUT
            WebSerialPro.println();
          #endif
        }  // end if dtsu666RegisterBlockActive
        dtsu666RegisterBlockIndex++;
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
void esp32Restart() {
  String mqttTopicID = "ESP32";
  String mqttTopicEntity = "Restart";
  String mqttPayload="1";
  bool mqttSuccess;

  Serial.println("ESP32 Restarting...");
  #ifdef OTA_OUTPUT
    WebSerialPro.println("ESP32 Restarting...");
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
  WiFi.disconnect();
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
    WebSerialPro.println("ESP32 Restarting...");
    delay(ESP32_RESTART_DELAY);
    esp32Restart();
  } else if(buffer == "Halt") {
    WebSerialPro.println("ESP32 Halt...");
    delay(ESP32_RESTART_DELAY);
    while(1);
  } else {
    WebSerialPro.println("Usage: \'Restart\' | \'Halt\'");
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttOutputConverterState() {
  for(unsigned short element = 0; element < DTSU666_MODBUS_CONVERTER_MQTT_SUBSCRIPTION_ELEMENTS; element++) {
    switch(element) {
      case 0: {  // DTSU666_Modbus_Converter_Modbus_Errors_0xE0
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[0];
        break;
      }  // end case
      case 1: {  // DTSU666_Modbus_Converter_Modbus_Errors_0xE1
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[1];
        break;
      }  // end case
      case 2: {  // DTSU666_Modbus_Converter_Modbus_Errors_0xE2
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[2];
        break;
      }  // end case
      case 3: {  // DTSU666_Modbus_Converter_Modbus_Errors_0xE3
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = modbusErrorCodes[3];
        break;
      }  // end case
      case 4: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveCounter;
        break;
      }  // end case
      case 5: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive_Received
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveReceivedCounter;
        break;
      }  // end case
      case 6: {  // DTSU666_Modbus_Converter_MQTT_Keep_Alive_Delayed
        mqttTopicEntity = String(dtsu666ModbusConverterMQTTSubscriptionsArray[element]).c_str();
        mqttPayloadData = mqttKeepAliveDelayedCounter;
        break;
      }  // end case
      default: {
        break;
      }  // end default
    }  // end switch element

    mqttTopicID = "DTSU666Converter";
    mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, mqttTopicEntity, mqttPayloadData, mqttRetained, mqttQoS);
  }  // end for element
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputInfoline() {
  #ifdef DEBUG_OUTPUT
    sprintf(msgString, "Smart Meter DTSU666 Prod - %s", String(actDateTime).c_str());
    Serial.println(msgString);
    
    sprintf(msgString, "Modbus Errors 0xE0|0xE1|0xE2|0xE3 [%lu | %lu | %lu | %lu]", modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]);
    Serial.println(msgString);
    
    sprintf(msgString, "MQTT KeepAlive|Received|Delayed [%lu | %lu | %lu]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    Serial.println(msgString);
    
    Serial.print("> / <");
  #endif
  
  #ifdef OTA_OUTPUT
    sprintf(msgString, "Smart Meter DTSU666 Prod - %s", String(actDateTime).c_str());
    WebSerialPro.println(msgString);
    
    sprintf(msgString, "Modbus Errors 0xE0|0xE1|0xE2|0xE3 [%lu | %lu | %lu | %lu]", modbusErrorCodes[MODBUS_ERROR_CODE_0XE0_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE1_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE2_INDEX], modbusErrorCodes[MODBUS_ERROR_CODE_0XE3_INDEX]);
    WebSerialPro.println(msgString);
    
    sprintf(msgString, "MQTT KeepAlive|Received|Delayed [%lu | %lu | %lu]", mqttKeepAliveCounter, mqttKeepAliveReceivedCounter, mqttKeepAliveDelayedCounter);
    WebSerialPro.println(msgString);
    
    WebSerialPro.println();
  #endif
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void mqttKeepAliveReceiver(String &inputTopic, String &inputPayload) {
  String mqttKeepAliveTopic;
  String mqttTopicID = "MQTT";
  String mqttTopicEntity = "Keep_Alive_DTSU666";
  
  mqttKeepAliveTopic = String(mqttTopicID + "/" + mqttTopicEntity).c_str();
  if(inputTopic == mqttKeepAliveTopic) {
    mqttKeepAliveReceivedCounter++;
    mqttKeepAliveDelayChrono.stop();
    
    #ifdef DEBUG_OUTPUT
      Serial.println("MQTT Message Receiver: " + inputTopic + " - " + inputPayload);
      Serial.print("> / <");
    #endif
    #ifdef OTA_OUTPUT
      WebSerialPro.println("MQTT Message Receiver: " + inputTopic + " - " + inputPayload);
      WebSerialPro.println();
    #endif
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------
void outputDTSU666Data(uint16_t dtsu666EntityResponseBufferOffset, char *dtsu666EntityDesriptor, char *dtsu666EntityDataType, float dtsu666EntityDataFactor, char *dtsu666EntityUnit, bool mqttActive, bool mqttRetained, uint16_t mqttQoS) {
  String mqttTopicID = "DTSU666";
  bool mqttSuccess;
  
  if(dtsu666EntityDataType == "CHAR") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    bufferValueChar = (bufferValueUInt16 & 0x00FF);  // 8 bit conversion
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %c", String(dtsu666EntityDesriptor).c_str(), bufferValueChar);
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %c", String(dtsu666EntityDesriptor).c_str(), bufferValueChar);
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      //mqttSuccess = mqttPublishTopicPayloadChar(mqttTopicID, dtsu666EntityDesriptor, bufferValueChar, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "U16") {
    bufferValueUInt16 = modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset);
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDesriptor).c_str(), bufferValueUInt16, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDesriptor).c_str(), bufferValueUInt16, String(dtsu666EntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt16(mqttTopicID, dtsu666EntityDesriptor, bufferValueUInt16, mqttRetained, mqttQoS);
    }
  } else if(dtsu666EntityDataType == "U32") {
    bufferValueUInt32 = ((modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset) << 16) + modbusNode.getResponseBuffer(dtsu666EntityResponseBufferOffset + 1));
    delay(MODBUS_DELAY);
    
    #ifdef DEBUG_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDesriptor).c_str(), bufferValueUInt32, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %u%s", String(dtsu666EntityDesriptor).c_str(), bufferValueUInt32, String(dtsu666EntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadUInt32(mqttTopicID, dtsu666EntityDesriptor, bufferValueUInt32, mqttRetained, mqttQoS);
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
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDesriptor).c_str(), bufferValueFloat16, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDesriptor).c_str(), bufferValueFloat16, String(dtsu666EntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat16(mqttTopicID, dtsu666EntityDesriptor, bufferValueFloat16, mqttRetained, mqttQoS);
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
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDesriptor).c_str(), bufferValueFloat32, String(dtsu666EntityUnit).c_str());
      Serial.println(msgString);
    #endif
    #ifdef OTA_OUTPUT
      sprintf(msgString, "%s: %0.2f%s", String(dtsu666EntityDesriptor).c_str(), bufferValueFloat32, String(dtsu666EntityUnit).c_str());
      WebSerialPro.println(msgString);
    #endif
    
    if(mqttActive) {
      mqttSuccess = mqttPublishTopicPayloadFloat32(mqttTopicID, dtsu666EntityDesriptor, bufferValueFloat32, mqttRetained, mqttQoS);
    }
  } else {
    #ifdef DEBUG_OUTPUT
      Serial.println("Unknown Type!");
    #endif
    #ifdef OTA_OUTPUT
      WebSerialPro.println("Unknown Type!");
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