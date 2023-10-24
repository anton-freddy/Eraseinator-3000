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
// #include <FTP.h>

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

struct webservervar
{
  int led_on;
  int motor_on;
};

int update_led();
void web_setup(void);
void web_loop(void);
void update_lidar_readings(int LiDAR1, int LiDAR2);
void sendCoordinatesToWeb(float xValue, float yValue, float xRobot, float yRobot);
void checkReconnectWiFi();

// webservervar server_var;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long WIFIpreviousMillis = 0;

String HTTP_SEND;
const char* SSID_cfg = "ANTONS-ZENBOOK";
const char* PASSWORD_cfg = "password";

void web_setup(void)
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin("ANTONS-ZENBOOK", "password");
  File config = LittleFS.open("/cfg.txt");
  config.find("SSID_cfg:<");
  SSID_cfg = config.readStringUntil('>').c_str();
  config.find("PASSWORD_cfg:<");
  PASSWORD_cfg = config.readStringUntil('>').c_str();
  config.close();

  delay(1000);

  Serial.println(SSID_cfg);
  Serial.println(PASSWORD_cfg);

  
  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    if (i > 100)
    {
      WiFi.softAP(ap_ssid, ap_password);
      Serial.println("");
      Serial.print("Network Created: ");
      Serial.println((String)ap_ssid + (String) ", " + (String)ap_password);
      Serial.print("IP address: ");
      Serial.println(WiFi.softAPIP());
      break;
    }
    delay(100);
    Serial.print(".");
    i++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    WIFIpreviousMillis = millis();
  }

  if (!MDNS.begin("Eraseinator3000"))
  {
    Serial.println("Error starting mDNS");
  }

  // Serve the HTML, JS, and CSS files
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // Handle WebSocket events
  // ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  //   if (type == WS_EVT_DATA)
  //   {
  //     AwsFrameInfo *info = (AwsFrameInfo *)arg;
  //     if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  //     {
  //       // Handle WebSocket text message
  //       String message = String((char *)data);
  //       Serial.println("WebSocket message received: " + message);

  //       // Process the message and send data to clients if needed
  //       // For example, send data to the xyPlane via WebSocket
  //       //ws.textAll(message);
  //     }
  //   }
  // });
  // Add WebSocket to server
  server.addHandler(&ws);
  // WebSerial.begin(&server2, "/serial");  // Start web serial
  // AsyncElegantOTA.begin(&server2, "admin", "admin"); // Start ElegantOTA
  // server2.begin();
  server.begin();

  Serial.println("HTTP server started");

  // FTP_setup();
  // Serial.println("FTP server started");
}

unsigned long WSpreviousMillis = 0;

void web_loop(void)
{
  // sendCoordinatesToWeb(random(-20,20),random(-20,20));
  if(millis() - WSpreviousMillis > 1000){
    WSpreviousMillis = millis();
    ws.cleanupClients();
  }

  // checkReconnectWiFi();
}

void update_lidar_readings(int LiDAR1, int LiDAR2)
{
}

void sendCoordinatesToWeb(float xValue, float yValue, float xRobot, float yRobot)
{
  xValue = xValue / 100;
  yValue = yValue / 100;
  xRobot = xRobot / 100;
  yRobot = yRobot / 100;
  // Create a message in the format "x,y"
  String message = String(xValue) + "," + String(yValue) + "," + String(xRobot) + "," + String(yRobot);

  // Send the message to all connected clients
  ws.textAll(message);

  // Serial.println("Sent coordinates: " + message);
}

void checkReconnectWiFi()
{
  unsigned long WIFIcurrentMillis = millis();
  if (WIFIcurrentMillis - WIFIpreviousMillis >= 100000)
  {
    WIFIpreviousMillis = WIFIcurrentMillis;
    if (WiFi.status() == WL_DISCONNECTED)
    {
      Serial.println("WiFi Disconnected");
      WiFi.disconnect();
      WiFi.begin(SSID_cfg, PASSWORD_cfg);
    }
  }
}