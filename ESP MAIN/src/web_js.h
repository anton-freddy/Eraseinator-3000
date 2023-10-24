
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketClient.h>
//#include <WebSocketsClient.h>
//#include <AsyncWebSocket.h>

// Web Info
// const char *ssid = "Rajput";
// const char *password = "pinksquash989";
const char *ssid = "Doofenschmirtz-Inc";
const char *password = "password";
// #define server_ip "192.168.55.40"
#define ws_path "/"
#define ws_host "192.168.1.3"
#define ws_port 81

WebSocketClient webSocketClient;
WiFiClient client;


// Function Prototypes
void wifiConnect();
void websocketConnect();



void client_setup()
{
	// Set up second core for Lidar

	Serial.begin(115200);
	wifiConnect();
	websocketConnect();
	//   data = "{\"x\":" + (String)(int)(x1) + ",\"y\":" +  (String)(int)(y1) + ",\"xloc\":" +  (String)(int)(x2) + ",\"yloc\":" +  (String)(int)(y2) + "}";

	//   webSocketClient.sendData(data);

}

void client_loop()
{
		//delay(50);

}

void wifiConnect()
{
	WiFi.begin(ssid, password);
	Serial.println("Connecting");
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to WiFi network: " + (String)ssid + " with IP Address: ");
	Serial.println(WiFi.localIP());
	return;
}

void websocketConnect()
{

	while (!client.connect(ws_host, 81))
	{
		Serial.println("Connection failed.");
		delay(500);
	}

	Serial.println("Connected");

	// Handshake with the server
	webSocketClient.path = ws_path;

	webSocketClient.host = ws_host;
	if (webSocketClient.handshake(client))
	{
		Serial.println("Handshake successful on port: " + (String)(ws_host));
	}
	else
	{
		Serial.println("Handshake failed.");
		ESP.restart();
	}
	delay(200);
	return;
}

void send_coordinates(float x1, float y1, float xRob, float yRob){

	String data, data1, data2;
		if ((String)x1 != (String)(xRob))
		{
			if (client.connected())
			{
				data1 = "{\"x\":" + (String)(float)(x1) + ",\"y\":" + (String)(float)(y1) + ",\"xloc\":" + (String)(float)(xRob) + ",\"yloc\":" + (String)(float)(yRob) + "}";
				webSocketClient.sendData(data1);
				       Serial.print("Sending: ");
				       Serial.println(data1);
			}
			else
			{
				Serial.println("Client disconnected.");
				websocketConnect();
			}
			// 			if (client.connected()
		}
}
