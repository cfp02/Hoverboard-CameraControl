#include <Arduino.h>
#include <WiFi.h>
#include "HoverboardESPNow.h"

// ===================== MAC Address =====================
// Receiver MAC address (Hoverboard 2, Solid wheels)
const uint8_t RECEIVER_MAC[6] = {0x24, 0x0A, 0xC4, 0x1D, 0x29, 0xA0}; 

// ===================== Globals =====================
HoverboardESPNow hoverboard;
String serialBuffer = ""; 

// ===================== Feedback Callback =====================
// Entirely Passive: If we get data from wheels, print it immediately.
void onFeedback(const SerialFeedback& feedback) {
  // Format: V:42.0V L:0 R:0 T:45.7C
  // Added standard CSV-like formatting for easier parsing if needed later
  // but keeping your specific format for now.
  char line[64];
  int len = snprintf(line, sizeof(line), 
    "V:%.1fV L:%d R:%d T:%.1fC\n",
    feedback.batVoltage / 100.0f,
    feedback.speedL_meas,
    feedback.speedR_meas,
    feedback.boardTemp / 10.0f
  );
  
  if (len > 0 && len < sizeof(line)) {
    Serial.write((const uint8_t*)line, len);
  }
}

// ===================== Setup =====================
void setup() {
  // 115200 is okay, but 500000 or 921600 is better for high-speed ROS 
  // if you change it here, remember to change it in your ROS node too.
  Serial.begin(115200);
  delay(1000);
  
  // Register feedback callback
  hoverboard.setFeedbackCallback(onFeedback);

  // Initialize ESP-Now
  if (!hoverboard.begin(RECEIVER_MAC)) {
    Serial.println("Error: ESP-Now Init Failed");
    while(1) delay(1000);
  }
  
  // Save to NVS (optional, but keeps connection robust)
  hoverboard.savePeerMacToNVS(RECEIVER_MAC, "hoverboard", "peerMac");
  
  // Blink LED or print small startup msg so you know it didn't bootloop
  Serial.println("--- PASSIVE BASE STATION READY ---");
}

// ===================== Loop =====================
void loop() {
  // Read Serial from Computer (ROS)
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for command termination (Newline)
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        
        // Expected Format: S:<speed>,T:<steer>
        // Example: S:100,T:50
        
        int sIdx = serialBuffer.indexOf("S:");
        int tIdx = serialBuffer.indexOf("T:");
        
        if (sIdx >= 0 && tIdx >= 0) {
          // --- 1. Parse Speed ---
          String speedStr = serialBuffer.substring(sIdx + 2, tIdx);
          if (speedStr.endsWith(",")) speedStr.remove(speedStr.length() - 1);
          float speedFloat = speedStr.toFloat();
          
          // --- 2. Parse Steer ---
          String steerStr = serialBuffer.substring(tIdx + 2);
          float steerFloat = steerStr.toFloat();
          
          // --- 3. Constrain & Convert ---
          // Multiplied by 10 as per your previous logic (-100.0 -> -1000)
          int speed = constrain((int)roundf(speedFloat * 10.0f), -1000, 1000);
          int steer = constrain((int)roundf(steerFloat * 10.0f), -1000, 1000);
          
          // --- 4. SEND IMMEDIATELY ---
          // No timers, no rate limits. If ROS sends 100 commands/sec, we try to send 100/sec.
          hoverboard.send(speed, steer);
          
          // Optional: Uncomment for debug, but better to keep silent for ROS
          // Serial.print("TX: "); Serial.print(speed); Serial.print(" "); Serial.println(steer);
          
        } else {
          // Only print error if buffer wasn't empty/junk
          // Serial.println("Err: fmt");
        }
        serialBuffer = ""; // Reset Buffer
      }
    } else {
      // Accumulate chars
      if (serialBuffer.length() < 64) {
        serialBuffer += c;
      }
    }
  }
  
  // No delay() here. Run as fast as possible.
}