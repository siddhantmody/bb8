#include "Arducam_Mega.h"
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>

const char* ssid = "TP-Link_8A8C"; //Router for GM Lab
const char* password = "12488674"; 

const int CS = 1;
Arducam_Mega myCAM(CS);
WebServer server(80);

void handle_stream() {
    Serial.println("Client connected to stream...");
    WiFiClient client = server.client();
    
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");

    while (client.connected()) {
        myCAM.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG);
        uint32_t len = myCAM.getTotalLength();
        
        if (len == 0) {
            Serial.println("Error: Camera captured 0 bytes");
            break;
        }

        client.print("--frame\r\n");
        client.print("Content-Type: image/jpeg\r\n");
        client.print("Content-Length: " + String(len) + "\r\n\r\n");

        uint8_t buffer[512]; // Smaller buffer is safer for C3 RAM
        while (len > 0) {
            size_t count = (len > sizeof(buffer)) ? sizeof(buffer) : len;
            myCAM.readBuff(buffer, count);
            client.write(buffer, count);
            len -= count;
            // VERY IMPORTANT: Keep the WiFi alive during the transfer
            yield(); 
        }
        client.print("\r\n");
        yield();
    }
    Serial.println("Client disconnected.");
}

void setup() {
    Serial.begin(115200);
    delay(1000); 

    Serial.println("Starting WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: http://");
    Serial.println(WiFi.localIP());

    // Initialize SPI
    SPI.begin(4, 5, 6, 1); 
    
    if (myCAM.begin() != CAM_ERR_SUCCESS) {
        Serial.println("Camera Init Failed!");
    } else {
        Serial.println("Camera Init Success!");
    }
    
    server.on("/stream", handle_stream);
    // Basic root page to test if the server is alive at all
    server.on("/", []() {
        server.send(200, "text/plain", "ESP32-C3 Server is LIVE. Go to /stream");
    });
    
    server.begin();
    Serial.println("HTTP Server Started.");
}

void loop() {
    server.handleClient();
    delay(1); 
}