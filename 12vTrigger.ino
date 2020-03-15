
/*
 * This is a custom circuit used to help control a modified Crestron CNAMPX-16X60. 
 * 
 * https://www.crestron.com/en-US/Products/Inactive/Discontinued/A-M/CNAMPX-16X60
 * 
 * This amp was modified with an Arduino on the main Crestnet board to trigger the "bypass" switch
 * about 5-6 seconds after receiving power to the Crestnet board. While that board is intended to 
 * take 24v DC in, 19v works just fine. This amplifier powers two different setups, my home theater which
 * uses an Emotiva XMC-1 as a source (which has 12v triggers), and my living room system which uses
 * an Echo Dot fed into a MiniDSP DDRC-24. 
 * 
 * This board is used to remotely control the 19v power to the Crestnet board, allowing wifi control
 * of the main power, while also taking in a 12v DC trigger from an Emotiva XMC-1, and audio sensing.
 * This will allow automatic operation with the home theater and living room sources, or manual overrides,
 * as well as reporting status to Home Assistant.
 * 
 * TODO: Add in capability for audio sensing
 * 
 * ----------------------------------------------------------------------------------
 *  ESP-12E Pinout:
 * ----------------------------------------------------------------------------------
 *                    ------------------------
 *                  --| Reset       D1 (TX0) |-- Serial TX/Prog RX
 *                  --| ADC         D3 (RX0) |-- Serial RX/Prog TX
 *              VCC --| CHPD        D4 (SCL) |--
 *                  --| D16         D5 (SDA) |--
 *   19v output FET --| D14 (SCK)         D0 |-- Bootloader (low - program, high - normal)
 *                  --| D12 (MISO)  D2 (TX1) |--
 *    3v trigger in --| D13 (MOSI)  D15 (SS) |-- GND (for normal startup)
 *                  --| VCC              GND |--
 *                    ------------------------
 * ----------------------------------------------------------------------------------
 * Notes:
 *  - Voltage divider circuit takes in 12v, using 100k and 33k ohm resistors to reduce the voltage to ~3v for input
 *  - 1000 ohm resistor on mosfet output, n-channel mosfet
 */
 
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "mqtt_config.h"

#define MQTT_STATE_TOPIC            "crestron/state"
#define MQTT_ATTRIBUTES_TOPIC       "crestron/attributes"
#define MQTT_COMMAND_TOPIC          "crestron/command"
#define WIFI_CLIENT_NAME            "Crestron Controller"
#define MDNS_NAME                   "crestron"

#define TRIGGER_PIN         13
#define MOSFET_PIN          14
#define NUM_SAMPLES         3 // number of samples before we consider a 12v trigger state change

const int pollRate = 1000; // how often we should read the value for the 12v trigger


long lastMsg = 0;
uint8_t stateChangeCount = 0;

// Used to track the state and the reason the amp is on
enum { CURRENT_STATE, TRIGGER, AUDIO, OVERRIDE };

boolean states[4] = {false, false, false, false};

MDNSResponder mdns;
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient mqttClient;
PubSubClient mqtt(mqttClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Hello! I'm here to check if there's a 12 volt trigger active.");

  pinMode(TRIGGER_PIN, INPUT);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, 0);

  WiFiManager wifiManager;
  if (!wifiManager.autoConnect(WIFI_CLIENT_NAME)) ESP.reset();

  // Start servers
  mdns.begin(MDNS_NAME);
  httpUpdater.setup(&server);
  server.begin();

  // Connect to MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqttReconnect();
  
}


void turnOn(uint8_t source){
  states[CURRENT_STATE] = true;
  states[source] = true;
  digitalWrite(MOSFET_PIN, 1);
  mqttPost();
}

void turnOffIfApplicable(){
  Serial.println("Turn off if applicable called!");
  Serial.print("states[TRIGGER] == ");
  Serial.println(states[TRIGGER] ? "TRIGGERED" : "OFF");
  Serial.print("states[AUDIO] == ");
  Serial.println(states[AUDIO] ? "AUDIOD" : "OFF");
  Serial.print("states[OVERRIDE] == ");
  Serial.println(states[OVERRIDE] ? "OVERRIDDEN" : "OFF");
  mqttPost();
  if(!states[TRIGGER] && !states[AUDIO] && !states[OVERRIDE])
    turnOff();
}

void turnOff(){
  states[CURRENT_STATE] = false;
  digitalWrite(MOSFET_PIN, 0);
  mqttPost();
}

void mqttPost(){
    Serial.println("Posting update to MQTT");
    DynamicJsonBuffer json;
    JsonObject &root = json.createObject();
    
    root["state"] = (states[CURRENT_STATE] ? "ON" : "OFF");
    root["trigger"] = (states[TRIGGER] ? "TRIGGERED" : "OFF");
    root["audio"] = (states[AUDIO] ? "AUDIOED" : "OFF");
    root["override"] = (states[OVERRIDE] ? "OVERRIDDEN" : "OFF");
    
    char buffer[root.measureLength() + 1];
    root.printTo(buffer, sizeof(buffer));
    mqtt.publish(MQTT_ATTRIBUTES_TOPIC, buffer, true);
    mqtt.publish(MQTT_STATE_TOPIC, (states[CURRENT_STATE] ? "ON" : "OFF"), true);
}

void mqttReconnect(){
  static long lastReconnect = 0;

  if (millis() - lastReconnect > 5000){
    if (mqtt.connect(WIFI_CLIENT_NAME, MQTT_USER, MQTT_PASS)){
      mqtt.subscribe(MQTT_COMMAND_TOPIC);
      
      mqttPost();

      lastReconnect = 0;
    }
    else lastReconnect = millis();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length){
  Serial.println("Callback triggered!");
  
  char payload_assembled[length];
  for (int i = 0; i < length; i++) payload_assembled[i] = (char)payload[i];
  Serial.print("Payload -- ");
  Serial.println(payload_assembled);
  
  if (strcmp(payload_assembled, "ON") == 0){
    turnOn(OVERRIDE);
    Serial.println("Turning on override");
  }
  else if (strcmp(payload_assembled, "OFF") == 0){
    states[OVERRIDE] = false;
    turnOffIfApplicable();
    Serial.println("Setting override off.");
  } else
    Serial.println("Didn't match checks");

}

void loop() {
 
   // Handle servers
   if (!mqtt.connected()) mqttReconnect();
   if (mqtt.connected()) mqtt.loop();
   server.handleClient();

   long now = millis();
   if (now - lastMsg > pollRate){
    lastMsg = now;
    Serial.print("Polling... ");

    // Check the states of the 12 volt trigger
    if(states[TRIGGER]){
      Serial.print(" current state is on and the pin");
      if(digitalRead(TRIGGER_PIN) == LOW){
        Serial.println(" is low!");
        if (++stateChangeCount > NUM_SAMPLES){
          states[TRIGGER] = false;
          turnOffIfApplicable();
          stateChangeCount = 0;
        }
      } else{ 
        Serial.println(" is high!");
        stateChangeCount = 0;
      }
    } else {
      Serial.print(" current state is off and the pin");
      if(digitalRead(TRIGGER_PIN) == HIGH){
        Serial.println(" is high!");
        if (++stateChangeCount > NUM_SAMPLES){
          turnOn(TRIGGER);
          states[TRIGGER] = true;
          stateChangeCount = 0;
        }
      } else{ 
        Serial.println(" is low!");
        stateChangeCount = 0;
      }
    }

    // TODO: Implement audio sensing
    
  }
}
