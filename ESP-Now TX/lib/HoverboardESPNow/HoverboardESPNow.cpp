#include "HoverboardESPNow.h"

// Static instance pointer for callback
HoverboardESPNow* HoverboardESPNow::_instance = nullptr;

// ===================== Constructor/Destructor =====================
HoverboardESPNow::HoverboardESPNow() 
  : _initialized(false), _peerAdded(false), _sequence(0), _feedbackCallback(nullptr) {
  memset(_peerMac, 0, 6);
  _instance = this; // Set instance pointer
}

HoverboardESPNow::~HoverboardESPNow() {
  if (_initialized) {
    esp_now_deinit();
  }
}

// ===================== CRC Calculation =====================
uint16_t HoverboardESPNow::calculateCRC(const uint8_t* data, size_t len, uint16_t crc) {
  while (len--) {
    crc ^= ((uint16_t)*data++) << 8;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ===================== Private Helpers =====================
bool HoverboardESPNow::_addPeer(const uint8_t mac[6]) {
  if (!_initialized) return false;
  
  // Remove existing peer if any
  if (_peerAdded) {
    esp_now_del_peer(_peerMac);
  }
  
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  
  esp_err_t result = esp_now_add_peer(&peer);
  if (result == ESP_OK) {
    memcpy(_peerMac, mac, 6);
    _peerAdded = true;
    return true;
  }
  
  return false;
}

// ===================== Public Methods =====================
bool HoverboardESPNow::begin(const uint8_t peerMac[6]) {
  if (_initialized) {
    // Already initialized, just update peer
    return setPeerMac(peerMac);
  }
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register receive callback if feedback callback is set
  if (_feedbackCallback != nullptr) {
    esp_now_register_recv_cb(_onEspNowRecv);
  }
  
  _initialized = true;
  return _addPeer(peerMac);
}

bool HoverboardESPNow::begin(const char* nvsNamespace, const char* nvsKey) {
  if (_initialized) {
    // Already initialized, try to load and set peer
    return loadPeerMacFromNVS(nvsNamespace, nvsKey) && setPeerMac(_peerMac);
  }
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register receive callback if feedback callback is set
  if (_feedbackCallback != nullptr) {
    esp_now_register_recv_cb(_onEspNowRecv);
  }
  
  _initialized = true;
  
  // Try to load MAC from NVS
  if (loadPeerMacFromNVS(nvsNamespace, nvsKey)) {
    return _addPeer(_peerMac);
  }
  
  // No MAC in NVS, but initialization succeeded
  return true;
}

bool HoverboardESPNow::begin() {
  if (_initialized) {
    return true; // Already initialized
  }
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  
  // Register receive callback if feedback callback is set
  if (_feedbackCallback != nullptr) {
    esp_now_register_recv_cb(_onEspNowRecv);
  }
  
  _initialized = true;
  return true;
}

bool HoverboardESPNow::setPeerMac(const uint8_t mac[6], bool saveToNVS, 
                                  const char* nvsNamespace, const char* nvsKey) {
  if (!_initialized) {
    return false;
  }
  
  bool success = _addPeer(mac);
  
  if (success && saveToNVS) {
    savePeerMacToNVS(mac, nvsNamespace, nvsKey);
  }
  
  return success;
}

bool HoverboardESPNow::send(int16_t speed, int16_t steer, uint8_t flags) {
  if (!_peerAdded) {
    return false;
  }
  
  ControlPacket packet{};
  packet.speed_pct = speed;
  packet.steer_pct = steer;
  packet.flags = flags;
  packet.seq = _sequence++;
  packet.crc = calculateCRC((const uint8_t*)&packet, sizeof(packet) - sizeof(packet.crc));
  
  esp_err_t result = esp_now_send(_peerMac, (const uint8_t*)&packet, sizeof(packet));
  return (result == ESP_OK);
}

bool HoverboardESPNow::isConnected() const {
  return _initialized && _peerAdded;
}

void HoverboardESPNow::getPeerMac(uint8_t mac[6]) const {
  memcpy(mac, _peerMac, 6);
}

bool HoverboardESPNow::loadPeerMacFromNVS(const char* nvsNamespace, const char* nvsKey) {
  Preferences prefs;
  if (!prefs.begin(nvsNamespace, true)) {
    return false;
  }
  
  uint8_t stored[6] = {0};
  size_t got = prefs.getBytes(nvsKey, stored, 6);
  prefs.end();
  
  if (got == 6 && memcmp(stored, "\0\0\0\0\0\0", 6) != 0) {
    memcpy(_peerMac, stored, 6);
    return true;
  }
  
  return false;
}

bool HoverboardESPNow::savePeerMacToNVS(const uint8_t mac[6], const char* nvsNamespace, const char* nvsKey) {
  Preferences prefs;
  if (!prefs.begin(nvsNamespace, false)) {
    return false;
  }
  
  bool success = (prefs.putBytes(nvsKey, mac, 6) == 6);
  prefs.end();
  
  if (success) {
    memcpy(_peerMac, mac, 6);
  }
  
  return success;
}

// ===================== Feedback Callback =====================
void HoverboardESPNow::setFeedbackCallback(FeedbackCallback callback) {
  _feedbackCallback = callback;
  
  // If already initialized and callback is being set, register it
  if (_initialized && _feedbackCallback != nullptr) {
    esp_now_register_recv_cb(_onEspNowRecv);
  }
}

void HoverboardESPNow::_onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // Check if we have an instance and callback
  if (_instance == nullptr || _instance->_feedbackCallback == nullptr) {
    return;
  }
  
  // Check if this is a feedback packet
  if (len == sizeof(FeedbackPacket)) {
    FeedbackPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    
    // Verify CRC16-CCITT (FeedbackPacket.crc is CRC16-CCITT of SerialFeedback structure)
    uint16_t crcCalc = calculateCRC((uint8_t*)&pkt.hb, sizeof(pkt.hb));
    
    if (crcCalc == pkt.crc) {
      // Call user callback with feedback data
      _instance->_feedbackCallback(pkt.hb);
    }
  }
}

