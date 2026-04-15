// #include "Arduino.h"
// #include "WiFi.h"
// #include "Audio.h"

// Audio audio;

// void setup() {
//     Serial.begin(115200);
    
//     WiFi.begin("YOUR_SSID", "YOUR_PASSWORD");
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("\nWiFi Connected!");

//     // On the C3, setting BCLK and LRC to -1 triggers the internal PWM mode
//     // The third argument (5) is your 'IN' pin on the Adafruit 3885
//     audio.setPinout(-1, -1, 18); 
    
//     audio.setVolume(21); // Max volume (0-21)

//     // Using a very stable MP3 stream
//     audio.connecttohost("http://icecast.unitedradio.it/Radio105.mp3");
// }

// void loop() {
//     audio.loop(); // This must run as fast as possible
// }

// Connect speaker 'IN' to GPIO 5
const int speakerPin = 18; 
const int freq = 440;      // Starting frequency in Hz
const int resolution = 8;  // 8-bit resolution

void setup() {
  // New API: ledcAttach(pin, frequency, resolution)
  // This replaces both ledcSetup and ledcAttachPin
  ledcAttach(speakerPin, freq, resolution); 
}

void loop() {
  // Play A4 (440Hz)
  ledcWriteTone(speakerPin, 440);
  delay(500);

  // Play A5 (880Hz)
  ledcWriteTone(speakerPin, 880);
  delay(500);

  // Stop sound (Frequency 0)
  ledcWriteTone(speakerPin, 0);
  delay(1000);
}