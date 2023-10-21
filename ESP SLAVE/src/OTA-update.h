/*
  Rui Santos
  Complete project details
   - Arduino IDE: https://RandomNerdTutorials.com/esp32-ota-over-the-air-arduino/
   - VS Code: https://RandomNerdTutorials.com/esp32-ota-over-the-air-vs-code/

  This sketch shows a Basic example from the AsyncElegantOTA library: ESP32_Async_Demo
  https://github.com/ayushsharma82/AsyncElegantOTA
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ESPmDNS.h>
#include <WebSerial.h>

#define LOCAL_NETWORK_MODE
// #define AP_MODE
// #define SLAVE_MODE

unsigned long ota_progress_millis = 0;

const char *URL = "Eraseinator3000-slave";

#ifdef SLAVE_MODE
const char *ssid = "Erasinator-3000";
const char *password = "Doofenshmirtz";
#endif

#ifdef AP_MODE
const char *ssid = "Erasinator-3000-Slave";
const char *password = "Doofenshmirtz";
#endif

#ifdef LOCAL_NETWORK_MODE
const char *ssid = "ANTONS-ZENBOOK";
const char *password = "password";
#endif

AsyncWebServer server(80);

//String HTTP_SEND;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void OTA_setup(void)
{
    Serial.begin(115200);
    

#ifdef AP_MODE
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.println("");
    Serial.print("Network Created: ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->redirect("/update"); });
#endif

#if defined LOCAL_NETWORK_MODE || defined SLAVE_MODE
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        i++;
        if(i > 20){
            Serial.println("Wifi Not Connected");
            break;
        }
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //IPAddress APip(WiFi.localIP());

    Serial.println("Before server.on");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->redirect("/update"); });
#endif

    if (!MDNS.begin("Eraseinator3000-slave"))
    {
        Serial.println("Error starting mDNS");
    }
    // WebSerial.begin(&server);
    Serial.println("Before AsyncElegantOTA.begin");
    ElegantOTA.begin(&server); // Start ElegantOTA
      ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
    // WebSerial.flush();
    Serial.println("After AsyncElegantOTA.begin");
    server.begin();
    Serial.println("HTTP server started");
}

unsigned long WIFIpreviousMillis = 0;

void OTA_loop(void){
    ElegantOTA.loop();

    unsigned long WIFIcurrentMillis = millis();
    if (WIFIcurrentMillis - WIFIpreviousMillis >= 10000)
    {
        WIFIpreviousMillis = WIFIcurrentMillis;
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi Disconnected");
            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
    }
}