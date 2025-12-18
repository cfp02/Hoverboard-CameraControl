#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "HoverboardESPNow.h"

// ===================== Configuration =====================

// This ESP32 Mac: d8:3b:da:45:61:cc // Base Station

// 1. WHEEL MAC (Receiver: Hoverboard 2, Solid wheels)
const uint8_t WHEEL_MAC[6] = {0x24, 0x0A, 0xC4, 0x1D, 0x29, 0xA0}; 

// 2. IMU MAC (ESP32-C3)
const uint8_t IMU_MAC[6]   = {0x8C, 0xD0, 0xB2, 0xA8, 0x58, 0x39}; 

// ===================== Data Structures =====================

// IMU Control Packet (Base Station -> IMU C3)
struct ImuControlPacket {
  uint8_t msgType;    // 1 = Serial Command
  uint8_t charData;   // e.g., 'e', 'q', 's'
};

// IMU Data Packet (IMU C3 -> Base Station)
struct ImuDataPacket {
  uint8_t msgType;    // 1 = Sideboard Data
  uint8_t len;      
  uint8_t payload[200];
};

// ===================== Globals =====================
HoverboardESPNow hoverboard;
String serialBuffer = ""; 

// ===================== Router Callback =====================
// We overwrite the library's default callback so we can handle multiple peers.
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  // Packet from Drivetrain
  if (memcmp(mac, WHEEL_MAC, 6) == 0) {
    // Hoverboard sends FeedbackPacket (contains SerialFeedback + CRC)
    if (len == sizeof(FeedbackPacket)) {
      FeedbackPacket pkt;
      memcpy(&pkt, incomingData, sizeof(pkt));
      
      // Verify CRC16-CCITT (same as library does)
      uint16_t crcCalc = HoverboardESPNow::calculateCRC((uint8_t*)&pkt.hb, sizeof(pkt.hb));
      
      if (crcCalc == pkt.crc) {
        // Access the SerialFeedback data from the packet
        const SerialFeedback* fb = &pkt.hb;
        
        // Format: [W] V:42.0V L:0 R:0 T:45.7C
        char line[64];
        int l = snprintf(line, sizeof(line), 
          "[W] V:%.1fV L:%d R:%d T:%.1fC\n",
          fb->batVoltage / 100.0f,
          fb->speedL_meas,
          fb->speedR_meas,
          fb->boardTemp / 10.0f
        );
        
        if (l > 0) Serial.write((const uint8_t*)line, l);
      }
    }
    return;
  }

  // Packet from IMU
  if (memcmp(mac, IMU_MAC, 6) == 0) {
    if (len >= 2) {
      const ImuDataPacket* pkt = (const ImuDataPacket*)incomingData;
      
      // Check for Sideboard Data Type (1)
      if (pkt->msgType == 1) {
        Serial.write("[I] ");
        // Write payload directly
        Serial.write(pkt->payload, pkt->len);
        // Note: The sideboard usually sends the newline, so we don't add one.
      }
    }
    return;
  }
}

// ===================== Helpers =====================
void addImuPeer() {
  // Wheels are added by hoverboard.begin(), but we add IMU manually
  if (esp_now_is_peer_exist(IMU_MAC)) return;
  
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, IMU_MAC, 6);
  peer.channel = 0; 
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[Error] Failed to add IMU Peer");
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // 1. Initialize Library (Sets up WiFi and Wheel Peer)
  if (!hoverboard.begin(WHEEL_MAC)) {
    Serial.println("Error: ESP-Now Init Failed");
    while(1) delay(1000);
  }
  
  // 2. Register Router Callback
  // This overwrites the callback set by hoverboard.begin()
  esp_now_register_recv_cb(onDataRecv);
  
  // 3. Add IMU Peer
  addImuPeer();
  
  // 4. Persistence (Optional, for wheels)
  hoverboard.savePeerMacToNVS(WHEEL_MAC, "hoverboard", "peerMac");

  Serial.println("--- ROUTER STATION READY ---");
  Serial.println("Send '[W] S:50,T:0' for Wheels");
  Serial.println("Send '[I] e' for IMU");
}

// ===================== Loop =====================
void loop() {
  // Read Serial from Computer (ROS)
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for command termination
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        serialBuffer.trim();
        
        // -------------------------------------------------
        // ROUTE TO WHEELS: "[W] S:100,T:50"
        // -------------------------------------------------
        if (serialBuffer.startsWith("[W]")) {
          int sIdx = serialBuffer.indexOf("S:");
          int tIdx = serialBuffer.indexOf("T:");
          
          if (sIdx >= 0 && tIdx >= 0) {
            String speedStr = serialBuffer.substring(sIdx + 2, tIdx);
            if (speedStr.endsWith(",")) speedStr.remove(speedStr.length() - 1);
            String steerStr = serialBuffer.substring(tIdx + 2);
            
            // Convert to +/- 1000 range
            int speed = constrain((int)roundf(speedStr.toFloat() * 10.0f), -1000, 1000);
            int steer = constrain((int)roundf(steerStr.toFloat() * 10.0f), -1000, 1000);
            
            // Use Library to Send
            hoverboard.send(speed, steer);
          }
        }
        
        // -------------------------------------------------
        // ROUTE TO IMU: "[I] e"
        // -------------------------------------------------
        else if (serialBuffer.startsWith("[I]")) {
          int spaceIdx = serialBuffer.indexOf(' ');
          // Ensure there is a character after the space
          if (spaceIdx >= 0 && spaceIdx + 1 < serialBuffer.length()) {
            char cmdChar = serialBuffer.charAt(spaceIdx + 1);
            
            ImuControlPacket pkt;
            pkt.msgType = 1;      
            pkt.charData = cmdChar;
            
            esp_now_send(IMU_MAC, (uint8_t*)&pkt, sizeof(pkt));
          }
        }
        
        serialBuffer = ""; // Reset Buffer
      }
    } else {
      // Accumulate
      if (serialBuffer.length() < 64) {
        serialBuffer += c;
      }
    }
  }
}