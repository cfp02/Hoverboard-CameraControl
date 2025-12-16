#include <Arduino.h>
#include <WiFi.h>
#include "HoverboardESPNow.h"

// ===================== MAC Address =====================
// Receiver MAC address 
// const uint8_t RECEIVER_MAC[6] = {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28}; // Hoverboard 1, Tires
const uint8_t RECEIVER_MAC[6] = {0x24, 0x0A, 0xC4, 0x1D, 0x29, 0xA0}; // Hoverboard 2, Solid wheels

// ===================== Globals =====================
HoverboardESPNow hoverboard;

int lastSpeed = 0;
int lastSteer = 0;
uint32_t lastSendTime = 0;
const unsigned SEND_INTERVAL_MS = 20; // Send at ~50Hz like the original

String serialBuffer = ""; // Buffer for accumulating serial input

// Feedback rate limiting
static uint32_t lastFeedbackPrint = 0;
static const uint32_t FEEDBACK_PRINT_INTERVAL_MS = 20; // Printing every 20ms for better integration for position    // 500; // Print every 500ms

// ===================== Feedback Callback =====================
void onFeedback(const SerialFeedback& feedback) {
  uint32_t now = millis();
  
  // Only print every 500ms
  if ((now - lastFeedbackPrint) >= FEEDBACK_PRINT_INTERVAL_MS) {
    // Build entire line as a compact string
    // Format: V:42.0V L:0 R:0 T:45.7C
    char line[48];
    int len = snprintf(line, sizeof(line), 
      "V:%.1fV L:%d R:%d T:%.1fC\n",
      feedback.batVoltage / 100.0f,
      feedback.speedL_meas,
      feedback.speedR_meas,
      feedback.boardTemp / 10.0f
    );
    
    // Use Serial.write to send entire string atomically (prevents corruption)
    if (len > 0 && len < sizeof(line)) {
      Serial.write((const uint8_t*)line, len);
    }
    
    lastFeedbackPrint = now;
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("ESP-NOW Passthrough");
  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());

  // Register feedback callback BEFORE initializing (so it gets registered properly)
  hoverboard.setFeedbackCallback(onFeedback);

  // Initialize ESP-Now with the receiver MAC
  if (!hoverboard.begin(RECEIVER_MAC)) {
    Serial.println("Failed to initialize ESP-Now!");
    while(1) delay(1000);
  }
  
  // Save to NVS for persistence
  hoverboard.savePeerMacToNVS(RECEIVER_MAC, "hoverboard", "peerMac");
  
  Serial.print("Using receiver MAC: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (RECEIVER_MAC[i] < 0x10) Serial.print("0");
    Serial.print(RECEIVER_MAC[i], HEX);
  }
  Serial.println();
  
  // Verify connection
  if (!hoverboard.isConnected()) {
    Serial.println("ERROR: Failed to add peer MAC!");
    while(1) delay(1000);
  }
  
  Serial.println("ESP-Now initialized and ready!");
  Serial.print("Receive callback registered: ");
  Serial.println(hoverboard.isConnected() ? "YES" : "NO");
  
  // Send a few initial packets to help with receiver auto-pairing
  Serial.println("Sending initial packets for pairing...");
  for (int i = 0; i < 5; i++) {
    hoverboard.send(0, 0);
    delay(50);
  }
  
  Serial.println("Ready! Send commands as: S:<speed>,T:<steer>");
  Serial.println("Example: S:50,T:30");
  Serial.println("Packets are sent continuously at ~50Hz");
  Serial.println("Waiting for feedback from hoverboard...");
  Serial.println("(If no 'ESPNow RX' messages appear, hoverboard may not be sending feedback)");
}

// ===================== Loop =====================
void loop() {
  uint32_t now = millis();
  
  // Non-blocking Serial input reading
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      // Process complete line
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        
        // Parse format: S:<speed>,T:<steer>
        int speed = 0, steer = 0;
        
        // Find speed value
        int sIdx = serialBuffer.indexOf("S:");
        int tIdx = serialBuffer.indexOf("T:");
        
        if (sIdx >= 0 && tIdx >= 0) {
          // Extract speed (between S: and comma or T:)
          String speedStr = serialBuffer.substring(sIdx + 2, tIdx);
          speedStr.trim();
          // Remove any trailing comma if present
          if (speedStr.endsWith(",")) {
            speedStr = speedStr.substring(0, speedStr.length() - 1);
          }
          speed = speedStr.toInt();
          
          // Extract steer (after T:)
          String steerStr = serialBuffer.substring(tIdx + 2);
          steerStr.trim();
          steer = steerStr.toInt();
          
          // Clamp values to [-100, 100]
          speed = constrain(speed, -100, 100);
          steer = constrain(steer, -100, 100);
          
          // Update last values
          lastSpeed = speed;
          lastSteer = steer;
          
          Serial.print("RX: speed=");
          Serial.print(speed);
          Serial.print(", steer=");
          Serial.print(steer);
          // Serial.print(" | Sending to hoverboard...");
          
          // Immediately send one packet to verify it works
          if (hoverboard.send(speed, steer)) {
            // Serial.println(" OK");
          } else {
            Serial.println(" FAILED");
          }
        } else {
          Serial.println("Invalid format! Use: S:<speed>,T:<steer>");
        }
        
        serialBuffer = ""; // Clear buffer
      }
      } else {
      // Add character to buffer
      serialBuffer += c;
      // Prevent buffer overflow
      if (serialBuffer.length() > 64) {
        serialBuffer = "";
        Serial.println("Buffer overflow, clearing...");
      }
    }
  }
  
  // Send packets continuously at fixed rate (like the original joystick)
  // This happens regardless of Serial activity
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    
    // Send the last received values (or 0,0 if nothing received yet)
    bool sent = hoverboard.send(lastSpeed, lastSteer);
    
    // Debug output every 1 second (50 sends)
    static uint32_t lastDebugTime = 0;
    if (now - lastDebugTime >= 1000) {
      lastDebugTime = now;
      Serial.print("TX: speed=");
      Serial.print(lastSpeed);
      Serial.print(", steer=");
      Serial.print(lastSteer);
      Serial.print(", status=");
      Serial.println(sent ? "OK" : "FAIL");
    }
  }
  
  delay(1); // Small delay
}

