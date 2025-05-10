//#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

// WiFi settings
const char* ssid = "AasmaDrone";
const char* password = "12345678";

// UDP settings
const char *udpAddress = "192.168.249.199";
const int udpPort = 4210;
const int maxPacketSize = 1400;       // Keep below typical MTU size

// LED for status indication
#define LED_PIN 33
#define LED_ON LOW
#define LED_OFF HIGH

// Camera settings
#define FRAME_SIZE FRAMESIZE_QVGA  // 320x240
#define JPEG_QUALITY 12            // Lower is better quality (0-63)

// Frame rate control
unsigned long previousFrameTime = 0;
const int frameInterval = 100;     // 10 fps (100ms between frames)

// UDP instance
WiFiUDP udp;

// Frame counter and statistics
uint32_t frameCount = 0;
uint32_t packetCount = 0;
uint32_t lastStatsTime = 0;
float fps = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ESP32-CAM Advanced UDP Streaming ---");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON); // LED on during setup
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    while(1) {
      digitalWrite(LED_PIN, LED_ON);
      delay(100);
      digitalWrite(LED_PIN, LED_OFF);
      delay(100);
    }
  }
  Serial.println("Camera initialized");
  
  // Print camera info
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("Camera sensor initialized: framesize %d, quality %d\n", 
                 s->status.framesize, s->status.quality);
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  // Wait for connection with timeout
  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(500);
    Serial.print(".");
    timeout--;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    while(1) {
      digitalWrite(LED_PIN, LED_ON);
      delay(500);
      digitalWrite(LED_PIN, LED_OFF);
      delay(500);
    }
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  
  // Start UDP
  udp.begin(udpPort);
  
  // Setup complete
  Serial.println("Streaming to: " + String(udpAddress) + ":" + String(udpPort));
  digitalWrite(LED_PIN, LED_OFF);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    digitalWrite(LED_PIN, LED_ON);
    WiFi.reconnect();
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    
    Serial.println("\nWiFi reconnected!");
    digitalWrite(LED_PIN, LED_OFF);
  }
  
  // Frame rate control
  unsigned long currentTime = millis();
  if (currentTime - previousFrameTime < frameInterval) {
    return;
  }
  previousFrameTime = currentTime;
  
  // Capture frame
  digitalWrite(LED_PIN, LED_ON);
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    digitalWrite(LED_PIN, LED_OFF);
    delay(100);
    return;
  }
  
  // Calculate FPS
  fps = 1000.0f / (currentTime - lastStatsTime);
  lastStatsTime = currentTime;
  
  // Increment frame counter
  frameCount++;
  
  // Send frame in chunks
  sendFrameInChunks(fb);
  
  // Return the frame buffer to the camera
  esp_camera_fb_return(fb);
  
  // Turn off LED
  digitalWrite(LED_PIN, LED_OFF);
  
  // Print statistics periodically
  if (frameCount % 30 == 0) {
    Serial.printf("Stats: %u frames, %u packets, %.1f FPS\n", 
                 frameCount, packetCount, fps);
  }
}

void sendFrameInChunks(camera_fb_t *fb) {
  // Calculate number of packets needed
  size_t totalBytes = fb->len;
  uint16_t totalPackets = (totalBytes + maxPacketSize - 12 - 1) / (maxPacketSize - 12);
  
  // Prepare header with info about frame
  uint8_t headerData[8];
  // Store frame counter as ID
  memcpy(headerData, &frameCount, 4);
  // Store total number of packets
  memcpy(headerData + 4, &totalPackets, 2);
  
  // Log frame info
  if (frameCount % 10 == 0) {
    Serial.printf("Frame %u: %u bytes in %u packets\n", frameCount, totalBytes, totalPackets);
  }
  
  // Send each packet
  for (uint16_t packetIndex = 0; packetIndex < totalPackets; packetIndex++) {
    // Calculate chunk size for this packet
    size_t offset = packetIndex * (maxPacketSize - 12);
    size_t packetDataSize = min(maxPacketSize - 12, (int)(totalBytes - offset));
    
    // Create packet: [frameCount(4)][totalPackets(2)][packetIndex(2)][packetSize(4)][data]
    uint8_t packetBuffer[12 + packetDataSize];
    
    // Copy header
    memcpy(packetBuffer, headerData, 6);
    
    // Add packet index
    memcpy(packetBuffer + 6, &packetIndex, 2);
    
    // Add packet size
    memcpy(packetBuffer + 8, &packetDataSize, 4);
    
    // Copy image data for this packet
    memcpy(packetBuffer + 12, fb->buf + offset, packetDataSize);
    
    // Send UDP packet
    udp.beginPacket(udpAddress, udpPort);
    udp.write(packetBuffer, sizeof(packetBuffer));
    bool success = udp.endPacket();
    
    if (success) {
      packetCount++;
    } else {
      Serial.println("Packet send failed");
    }
    
    // Small delay to prevent UDP buffer overflow
    delay(1);
  }
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
