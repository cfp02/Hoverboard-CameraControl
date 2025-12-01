# HoverboardESPNow Library

A simple, clean library for sending control commands to a hoverboard receiver ESP32 via ESP-Now protocol.

## Features

- **Simple API**: Just initialize and send speed/steer values
- **MAC Address Management**: Automatic persistence to NVS (Non-Volatile Storage)
- **Packet Format**: Compatible with existing hoverboard receiver firmware
- **Board Agnostic**: Works on any ESP32 variant (ESP32, ESP32-S2, ESP32-S3, etc.)
- **Minimal Dependencies**: Only requires ESP-Now, WiFi, and Preferences

## Installation

Place this library in your PlatformIO project's `lib/` directory:

```
your-project/
├── lib/
│   └── HoverboardESPNow/
│       ├── HoverboardESPNow.h
│       ├── HoverboardESPNow.cpp
│       └── README.md
├── src/
└── platformio.ini
```

## Quick Start

```cpp
#include "HoverboardESPNow.h"

// Receiver MAC address
const uint8_t RECEIVER_MAC[6] = {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28};

HoverboardESPNow hoverboard;

void setup() {
  Serial.begin(115200);
  
  // Initialize with MAC address
  if (!hoverboard.begin(RECEIVER_MAC)) {
    Serial.println("Failed to initialize!");
    while(1) delay(1000);
  }
  
  Serial.println("Ready!");
}

void loop() {
  // Send speed and steer values (both in range [-100, 100])
  hoverboard.send(50, 30);  // 50% speed, 30% steer
  
  delay(20);  // Send at ~50Hz
}
```

## API Reference

### Constructor

```cpp
HoverboardESPNow();
```

Creates a new HoverboardESPNow instance.

### Initialization

```cpp
// Initialize with MAC address directly
bool begin(const uint8_t peerMac[6]);

// Initialize and load MAC from NVS
bool begin(const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");

// Initialize without MAC (set later with setPeerMac)
bool begin();
```

### Sending Commands

```cpp
// Send speed and steer values
bool send(int16_t speed, int16_t steer, uint8_t flags = 0);
```

- `speed`: Speed percentage in range [-100, 100]
- `steer`: Steer percentage in range [-100, 100]
- `flags`: Optional flags byte (bit 0: invert speed, bit 1: invert steer, bit 2: log curve)

### MAC Address Management

```cpp
// Set peer MAC address
bool setPeerMac(const uint8_t mac[6], bool saveToNVS = false, 
                const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");

// Get current peer MAC
void getPeerMac(uint8_t mac[6]) const;

// Load MAC from NVS
bool loadPeerMacFromNVS(const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");

// Save MAC to NVS
bool savePeerMacToNVS(const uint8_t mac[6], const char* nvsNamespace = "hoverboard", const char* nvsKey = "peerMac");
```

### Status

```cpp
// Check if initialized and peer is connected
bool isConnected() const;
```

## Usage Examples

### Example 1: Direct MAC Address

```cpp
const uint8_t RECEIVER_MAC[6] = {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28};
HoverboardESPNow hoverboard;

void setup() {
  hoverboard.begin(RECEIVER_MAC);
}

void loop() {
  hoverboard.send(0, 0);
  delay(20);
}
```

### Example 2: Load MAC from NVS

```cpp
HoverboardESPNow hoverboard;

void setup() {
  // Try to load MAC from NVS, fallback to default if not found
  if (!hoverboard.begin("hoverboard", "peerMac")) {
    // If no MAC in NVS, set a default one
    const uint8_t DEFAULT_MAC[6] = {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28};
    hoverboard.setPeerMac(DEFAULT_MAC, true);  // Save to NVS
  }
}

void loop() {
  hoverboard.send(0, 0);
  delay(20);
}
```

### Example 3: Serial Input (for ROS integration)

```cpp
HoverboardESPNow hoverboard;

void setup() {
  Serial.begin(115200);
  hoverboard.begin();  // Initialize without MAC
  // Set MAC later or load from NVS
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    // Parse input (e.g., "S:50,T:30")
    int speed = 0, steer = 0;
    if (sscanf(input.c_str(), "S:%d,T:%d", &speed, &steer) == 2) {
      speed = constrain(speed, -100, 100);
      steer = constrain(steer, -100, 100);
      hoverboard.send(speed, steer);
    }
  }
  delay(10);
}
```

## Packet Format

The library sends packets in the following format (matches receiver expectations):

```cpp
struct ControlPacket {
  int16_t speed_pct;  // [-100, 100]
  int16_t steer_pct;  // [-100, 100]
  uint8_t flags;      // Configuration flags
  uint8_t seq;        // Sequence number (auto-incremented)
  uint16_t crc;       // CRC16-CCITT checksum (auto-calculated)
};
```

## Notes

- The library automatically handles sequence numbers and CRC calculation
- Recommended send rate: 20-50Hz (20-50ms between sends)
- The library is stateless regarding input processing - you handle all input reading and processing in your application code
- MAC addresses are stored in NVS for persistence across reboots

## Compatibility

- Compatible with existing hoverboard receiver firmware
- Works with ESP32, ESP32-S2, ESP32-S3, and other ESP32 variants
- PlatformIO compatible

