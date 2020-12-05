#include <Arduino.h>
/*
DMXGW - A simple E1.31 to DMX gateway application

Copyright (c) 2020 Nash Kaminski

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ArduinoOTA.h>
#include <WiFiManager.h>

// UI 
#if defined(ESP8266)
  /* ESP8266 Dependencies for UI */
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#elif defined(ESP32)
  /* ESP32 Dependencies for UI*/
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>

// DMX I/O
#include <ESPAsyncE131.h>
#include <HardwareSerial.h>

#define LED_PIN 13 // Connection Status LED

#define UNIVERSE 1       // First DMX Universe to listen for
#define UNIVERSE_COUNT 1 // Total number of Universes to listen for, starting at UNIVERSE

#define DMXSPEED 250000
#define DMXFORMAT SERIAL_8N2
#define DMXBREAKUSEC 88

// WifiManager for initial config
WiFiManager wifiManager;

// Sparkfun DMX shield pinout
int rxPin = 16;
int txPin = 17;
boolean connected = false;
HardwareSerial DMXSerial(2);

// ESPAsyncE131 instance with UNIVERSE_COUNT buffer slots
ESPAsyncE131 e131(UNIVERSE_COUNT);

// UI Initialization
bool uiRunning = false;
bool manualMode = false;
uint8_t manualBrightnessValue = 0;
AsyncWebServer uiserver(80);
ESPDash dashboard(&uiserver);
Card manualButton(&dashboard, BUTTON_CARD, "Manual Control");
Card manualBrightness(&dashboard, SLIDER_CARD, "Brightness", "", 0, 255);
Card resetButton(&dashboard, BUTTON_CARD, "Reset WiFi Settings");
Card rebootButton(&dashboard, BUTTON_CARD, "Reboot System");

// Internal state
// DMX buffer, 1 start code and 512 channels
uint8_t dmxBuffer[513];
e131_packet_t packet;


void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_CONNECTED:
    Serial.println(F("Connected to access point"));
    connected = true;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println(F("Disconnected from WiFi access point"));
    connected = false;
    break;

  default:
    break;
  }
}

void sendDMX(const uint8_t *data, size_t len)
{
  pinMatrixOutDetach(txPin, false, false); //Detach UART from IO Pin
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
  delayMicroseconds(DMXBREAKUSEC);
  digitalWrite(txPin, HIGH); //4 Us Mark After Break
  delayMicroseconds(1);
  pinMatrixOutAttach(txPin, U2TXD_OUT_IDX, false, false);
  DMXSerial.write(data, len);
  DMXSerial.flush();
}

void setup()
{
  // WiFi initializaion
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFi.setSleep(false); // No power management, else terrible latency!
  WiFi.onEvent(WiFiEvent);
  wifiManager.setConnectTimeout(60);
  wifiManager.setConfigPortalTimeout(300);

  // I/O initialization
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  DMXSerial.begin(DMXSPEED, DMXFORMAT, rxPin, txPin); //Initialize the Serial port
  
  // OTA update support
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println(F("OTA: Auth Failed"));
        else if (error == OTA_BEGIN_ERROR)
          Serial.println(F("OTA: Begin Failed"));
        else if (error == OTA_CONNECT_ERROR)
          Serial.println(F("OTA: Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println(F("OTA: Receive Failed"));
        else if (error == OTA_END_ERROR)
          Serial.println(F("OTA: End Failed"));
      });
  ArduinoOTA.begin();

  // UI initialization
  manualButton.attachCallback([&](bool value){
    manualMode = value;
    manualButton.update(value);
    dashboard.sendUpdates();
  });
  manualBrightness.attachCallback([&](int value){
    manualBrightnessValue = value;
    manualBrightness.update(value);
    dashboard.sendUpdates();
  });
  rebootButton.attachCallback([&](bool value){
    rebootButton.update(value);
    dashboard.sendUpdates();
    ESP.restart();
  });
  resetButton.attachCallback([&](bool value){
    resetButton.update(value);
    dashboard.sendUpdates();
    WiFi.disconnect(true, true);
    ESP.restart();
  });
  // Update UI once on init
  manualButton.update(manualMode);
  manualBrightness.update(manualBrightnessValue);
  dashboard.sendUpdates();
  
  // Choose one to begin listening for E1.31 data
  if (e131.begin(E131_UNICAST)) // Listen via Unicast
                                // if (e131.begin(E131_MULTICAST, UNIVERSE, UNIVERSE_COUNT))   // Listen via Multicast
    Serial.println(F("Listening for data..."));
  else
    Serial.println(F("*** e131.begin failed ***"));
}

void loop()
{
  // Externalize connected status via the onboard LED
  digitalWrite(LED_PIN, connected);

  // If not connected, then try to reconnect
  if (!connected)
  {
    // Stop UI server to free port 80 for WifiManager
    uiserver.end();
    uiRunning = false;
    // Try to connect, if not successful in 1 min then serve config portal for 5 min then reboot
    if( !wifiManager.autoConnect("DMXGW"))
      ESP.restart();
  }
  if (connected && !uiRunning) {
    // Otherwise if we are connected but the UI server is stopped, start it
    // UI server conflicts with WifiManager therefore only one may be active at a time
    uiserver.begin();
    uiRunning = true;
  }

  // OTA Handling
  ArduinoOTA.handle();

  // Dequeue e1.31 packet and action the most recent
  while (!e131.isEmpty())
  {
    e131.pull(&packet); // Pull packet from ring buffer

#ifdef DEBUG
    Serial.printf("Universe %u / %u Channels | Packet#: %u / Errors: %u / CH1: %u",
                  htons(packet.universe),                 // The Universe for this packet
                  htons(packet.property_value_count) - 1, // Start code is ignored, we're interested in dimmer data
                  e131.stats.num_packets,                 // Packet counter
                  e131.stats.packet_errors,               // Packet error counter
                  packet.property_values[1]);             // Dimmer data for Channel 1
    Serial.println();
#endif
    if (!manualMode)
      memcpy(dmxBuffer, packet.property_values, min((uint16_t)513, htons(packet.property_value_count)));
  }

  // Handle manual override mode
  if (manualMode){
    dmxBuffer[0] = 0x00;
    memset(dmxBuffer+1, manualBrightnessValue, sizeof(dmxBuffer)-1);
  }
  
  // Always write a DMX frame
  sendDMX(dmxBuffer, sizeof(dmxBuffer));
}