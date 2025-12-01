#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <stdint.h>
#include <stddef.h>

// ===================== Packet Format =====================
// Matches the exact format expected by the receiver ESP32
struct __attribute__((packed)) ControlPacket {
  int16_t speed_pct;  // Speed percentage [-100, 100]
  int16_t steer_pct;  // Steer percentage [-100, 100]
  uint8_t flags;      // Flags byte (bit 0: inv speed, bit 1: inv steer, bit 2: log curve)
  uint8_t seq;        // Sequence number
  uint16_t crc;       // CRC16-CCITT checksum
};

// ===================== HoverboardESPNow Class =====================
class HoverboardESPNow {
public:
  // Constructor
  HoverboardESPNow();
  
  // Destructor
  ~HoverboardESPNow();
  
  // Initialize ESP-Now with a specific peer MAC address
  // Returns true on success, false on failure
  bool begin(const uint8_t peerMac[6]);
  
  // Initialize ESP-Now and load peer MAC from NVS
  // Uses the provided namespace and key (defaults: "hoverboard", "peerMac")
  // Returns true on success, false on failure
  bool begin(const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");
  
  // Initialize ESP-Now without loading from NVS (peer must be set later)
  // Returns true on success, false on failure
  bool begin();
  
  // Set the peer MAC address (must call after begin() if not provided in begin())
  // Optionally save to NVS
  // Returns true on success, false on failure
  bool setPeerMac(const uint8_t mac[6], bool saveToNVS = false, 
                  const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");
  
  // Send a control packet with speed and steer values
  // speed: [-100, 100] speed percentage
  // steer: [-100, 100] steer percentage
  // flags: optional flags byte (defaults to 0)
  // Returns true on success, false on failure
  bool send(int16_t speed, int16_t steer, uint8_t flags = 0);
  
  // Check if ESP-Now is initialized and peer is added
  bool isConnected() const;
  
  // Get the current peer MAC address
  void getPeerMac(uint8_t mac[6]) const;
  
  // Load peer MAC from NVS
  // Returns true if MAC was loaded, false otherwise
  bool loadPeerMacFromNVS(const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");
  
  // Save peer MAC to NVS
  // Returns true on success, false on failure
  bool savePeerMacToNVS(const uint8_t mac[6], const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");
  
  // Calculate CRC16-CCITT for a packet (public for testing/external use)
  static uint16_t calculateCRC(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

private:
  bool _initialized;
  bool _peerAdded;
  uint8_t _peerMac[6];
  uint8_t _sequence;
  
  // Internal helper to add peer
  bool _addPeer(const uint8_t mac[6]);
};

