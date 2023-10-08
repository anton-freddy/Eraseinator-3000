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
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WebSerial.h>
//#include <FTP.h>


#define LOCAL_NETWORK_MODE
#define AP_MODE

#ifdef AP_MODE
const char *ap_ssid = "Eraseinator-3000";
const char *ap_password = "password";
#endif

#ifdef LOCAL_NETWORK_MODE
const char *ssid = "ANTONS-ZENBOOK";
const char *password = "password";
#endif

struct webservervar {
  int led_on;
  int motor_on;
};



int update_led();
void web_setup(void);
void web_loop(void);
void update_lidar_readings(int LiDAR1, int LiDAR2);
void sendCoordinatesToWeb(float xValue, float yValue);

//webservervar server_var;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



String HTTP_SEND;

void web_setup(void)
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  File config = LittleFS.open("/cfg.txt");
  config.find("SSID:<");
  String SSID = config.readStringUntil('>');
  config.find("PASSWORD:<");
  String PASSWORD = config.readStringUntil('>');

  Serial.println(SSID);
  Serial.println(PASSWORD);


WiFi.begin(SSID, PASSWORD);
int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    if(i > 1000){
        WiFi.softAP(ap_ssid, ap_password);
        Serial.println("");
        Serial.print("Network Created: ");
        Serial.println(ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.softAPIP());
        break;
    }
    delay(10);
    Serial.print(".");
    i++;
  }
  if(WiFi.status() == WL_CONNECTED){
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(ssid);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
  }


  if(!MDNS.begin("Eraseinator3000")) {
     Serial.println("Error starting mDNS");
}

    // Serve the HTML, JS, and CSS files
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Handle WebSocket events
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA)
    {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
      {
        // Handle WebSocket text message
        String message = String((char *)data);
        Serial.println("WebSocket message received: " + message);

        // Process the message and send data to clients if needed
        // For example, send data to the xyPlane via WebSocket
        //ws.textAll(message);
      }
    }
  });
  // Add WebSocket to server
  server.addHandler(&ws);
  WebSerial.begin(&server, "/serial");  // Start web serial
  AsyncElegantOTA.begin(&server, "admin", "admin"); // Start ElegantOTA
  server.begin();
  
  Serial.println("HTTP server started");

  // FTP_setup();
  // Serial.println("FTP server started");
}

void web_loop(void)
{
  sendCoordinatesToWeb(random(-20,20),random(-20,20));
  ws.cleanupClients();

}

void update_lidar_readings(int LiDAR1, int LiDAR2){

}


void sendCoordinatesToWeb(float xValue, float yValue)
{
  // Create a message in the format "x,y"
  String message = String(xValue) + "," + String(yValue);

  // Send the message to all connected clients
  ws.textAll(message);

  //Serial.println("Sent coordinates: " + message);
}