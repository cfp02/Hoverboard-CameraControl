// Simple example showing how to use HoverboardESPNow library
// This demonstrates basic usage for a serial-based input system

#include <Arduino.h>
#include "HoverboardESPNow.h"

// Example MAC address - replace with your receiver's MAC
const uint8_t RECEIVER_MAC[6] = {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28};

HoverboardESPNow hoverboard;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Initializing Hoverboard ESP-Now...");
  
  // Option 1: Initialize with MAC address directly
  if (!hoverboard.begin(RECEIVER_MAC)) {
    Serial.println("Failed to initialize ESP-Now!");
    while (1) delay(1000);
  }
  
  // Option 2: Initialize and load MAC from NVS (if previously saved)
  // if (!hoverboard.begin("hoverboard", "peerMac")) {
  //   Serial.println("Failed to initialize ESP-Now!");
  //   while (1) delay(1000);
  // }
  
  // Option 3: Initialize without MAC, set it later
  // if (!hoverboard.begin()) {
  //   Serial.println("Failed to initialize ESP-Now!");
  //   while (1) delay(1000);
  // }
  // hoverboard.setPeerMac(RECEIVER_MAC, true); // true = save to NVS
  
  Serial.println("ESP-Now initialized successfully!");
  Serial.print("Peer MAC: ");
  uint8_t mac[6];
  hoverboard.getPeerMac(mac);
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();
}

void loop() {
  // Example: Read speed and steer from Serial
  // Format: "S:50,T:30\n" or similar
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Simple parsing example (you'd want more robust parsing)
    int speed = 0, steer = 0;
    if (sscanf(input.c_str(), "S:%d,T:%d", &speed, &steer) == 2) {
      // Clamp values to [-100, 100]
      speed = constrain(speed, -100, 100);
      steer = constrain(steer, -100, 100);
      
      // Send packet
      if (hoverboard.send(speed, steer)) {
        Serial.print("Sent: Speed=");
        Serial.print(speed);
        Serial.print(", Steer=");
        Serial.println(steer);
      } else {
        Serial.println("Send failed!");
      }
    }
  }
  
  // Or send at a fixed rate (e.g., 50Hz = every 20ms)
  static uint32_t lastSend = 0;
  if (millis() - lastSend >= 20) {
    lastSend = millis();
    
    // Example: send some test values
    // hoverboard.send(0, 0);
  }
  
  delay(10);
}

