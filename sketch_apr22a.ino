#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

// --- Configuration ---
const char* ssid = "TP-Link_8A8C";
const char* password = "12488674";

WebServer server(80);
uint8_t imgBuffer[35000]; 

void handle_stream() {
  auto client = server.client();
  if (!client) return;

  Serial.println("[SERVER] Connection started.");

  // 1. Initial MJPEG Header (Note the exact spacing)
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  client.print("\r\n"); 

  while (client.connected()) {
    // Look for the "IMAGE" sync header from OpenMV
    if (Serial1.available() >= 5) {
      char header[6];
      Serial1.readBytes(header, 5);
      header[5] = '\0';

      if (strcmp(header, "IMAGE") == 0) {
        // Read the 4-byte size header
        while (Serial1.available() < 4) { yield(); }
        uint32_t imgSize = 0;
        Serial1.readBytes((char*)&imgSize, 4);

        if (imgSize > 0 && imgSize < 35000) {
          int bytesRead = 0;
          unsigned long start = millis();
          
          // Collect JPEG bytes
          while (bytesRead < imgSize && (millis() - start < 500)) {
            if (Serial1.available()) {
              imgBuffer[bytesRead] = Serial1.read();
              bytesRead++;
              if (bytesRead % 512 == 0) yield(); 
            }
          }

          // 2. Browser Frame Packaging
          if (bytesRead == imgSize) {
            client.print("--frame\r\n");
            client.print("Content-Type: image/jpeg\r\n");
            client.printf("Content-Length: %u\r\n", imgSize);
            client.print("\r\n"); // The critical empty line before binary data
            
            client.write(imgBuffer, imgSize);
            
            client.print("\r\n"); // End of frame line
          }
        }
      }
    }
    yield(); // Handle WiFi background tasks
  }
  Serial.println("[SERVER] Connection closed.");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Serial1 on G1 (RX)
  Serial1.begin(115200, SERIAL_8N1, 1, -1); 
  Serial1.setRxBufferSize(8192); 

  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[WIFI] Connected!");
  Serial.print("[WIFI] Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  server.on("/stream", handle_stream);
  server.begin();
}

void loop() {
  server.handleClient();
}