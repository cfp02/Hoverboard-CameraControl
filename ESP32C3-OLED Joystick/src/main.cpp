#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "OLEDScreen.h"

// ===================== Hoverboard List =====================
struct BoardEntry {
  const char* name;
  uint8_t mac[6];
};
static const BoardEntry BOARDS[] = {
  {"H1", {0xAC, 0x67, 0xB2, 0x53, 0x86, 0x28}},
  {"H2", {0xE8, 0xDB, 0x84, 0x03, 0xF1, 0x80}},
};
static const size_t NUM_BOARDS = sizeof(BOARDS) / sizeof(BOARDS[0]);

// ===================== Pins =====================
#define BUTTON_PIN  GPIO_NUM_0
#define POT_X_PIN   GPIO_NUM_2
#define POT_Y_PIN   GPIO_NUM_1
#define LED_PIN     GPIO_NUM_8

// ===================== Packet Format =====================
struct __attribute__((packed)) ControlPacket {
  int16_t speed_pct;
  int16_t steer_pct;
  uint8_t flags;
  uint8_t seq;
  uint16_t crc;
};
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  while (len--) {
    crc ^= ((uint16_t)*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// ===================== Globals =====================
OLEDScreen screen;
Preferences prefs;

enum UiState { RUN, MENU, SELECT };
UiState ui = RUN;

int centerX = 2048, centerY = 2048;
int deadzone = 180;
bool optInvSpeed = false;
bool optInvSteer = false;
bool optLog = true;
int  optSpeedMode = 0; // 0=High, 1=Med, 2=Low
int  optTurnMod = 0; // 0=Off, 1=1/2, 2=1/4, 3=Dyn
int  optRampRate = 0; // 0=Off, 1=Hi, 2=Med, 3=Low
const float SPEED_LIMITS[3] = {100.0f, 60.0f, 30.0f};
const float RAMP_RATES[3] = {10.0f, 5.0f, 2.0f}; // % per cycle (Hi, Med, Low)

uint8_t seq = 0;
uint32_t tLastSend = 0, tLastOLED = 0;
const unsigned SEND_MS = 20;
const unsigned OLED_MS = 100;

int lastSpeed = 0;
int lastSteer = 0;

bool btnPrev = false;
uint32_t tBtnDown = 0;
const unsigned LONG_PRESS_MENU_MS   = 5000;
const unsigned LONG_PRESS_SELECT_MS = 8000;

bool menuShown = false;
bool selectShown = false;
bool buttonReleasedSinceEntry = false;
bool justEnteredViaLongPress = false;
bool justEnteredViaMacSelect = false;
bool justExitedFromMenu = false;

uint8_t PEER_MAC[6] = {0};
int selectIdx = 0;
bool peerAdded = false;

bool xMoved = false;

// ===================== Helpers =====================
static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline bool buttonPressed() { return digitalRead(BUTTON_PIN) == LOW; }

static float adcToPercent(int raw, int center) {
  int d = raw - center;
  if (abs(d) <= deadzone) return 0.0f;
  int span = (4095 / 2) - deadzone;
  float pct = (float)d / (float)span * 100.0f;
  return clampf(pct, -100.0f, 100.0f);
}
static float curveLog(float pct) {
  float x = fabsf(pct) / 100.0f;
  float y = powf(x, 1.5f) * 100.0f;
  return (pct < 0.0f) ? -y : y;
}
static uint8_t flagsByte() {
  uint8_t f = 0;
  if (optInvSpeed) f |= 1 << 0;
  if (optInvSteer) f |= 1 << 1;
  if (optLog)      f |= 1 << 2;
  return f;
}

// ===================== ESPNOW =====================
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_send_cb(nullptr);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) == ESP_OK) peerAdded = true;
  return peerAdded;
}
bool sendPacket(int speedPct, int steerPct) {
  if (!peerAdded) return false;
  ControlPacket p{};
  p.speed_pct = (int16_t)speedPct;
  p.steer_pct = (int16_t)steerPct;
  p.flags     = flagsByte();
  p.seq       = seq++;
  p.crc       = crc16_ccitt((const uint8_t*)&p, sizeof(p) - sizeof(p.crc));
  return esp_now_send(PEER_MAC, (const uint8_t*)&p, sizeof(p)) == ESP_OK;
}

// ===================== Settings =====================
void loadSettings() {
  prefs.begin("joy", true);
  optInvSpeed  = prefs.getBool("invSpd", false);
  optInvSteer  = prefs.getBool("invStr", false);
  optLog       = prefs.getBool("log", true);
  optSpeedMode = prefs.getInt("spdMode", 0);
  optTurnMod   = prefs.getInt("turnMod", 0);
  optRampRate  = prefs.getInt("rampRate", 0);
  centerX      = prefs.getInt("cx", 2048);
  centerY      = prefs.getInt("cy", 2048);
  deadzone     = prefs.getInt("dz", 180);
  uint8_t stored[6] = {0};
  size_t got = prefs.getBytes("peerMac", stored, 6);
  prefs.end();

  if (got == 6 && memcmp(stored, "\0\0\0\0\0\0", 6) != 0)
    memcpy(PEER_MAC, stored, 6);
  else if (NUM_BOARDS > 0)
    memcpy(PEER_MAC, BOARDS[0].mac, 6);
}
void saveSettings() {
  prefs.begin("joy", false);
  prefs.putBool("invSpd", optInvSpeed);
  prefs.putBool("invStr", optInvSteer);
  prefs.putBool("log", optLog);
  prefs.putInt ("spdMode", optSpeedMode);
  prefs.putInt ("turnMod", optTurnMod);
  prefs.putInt ("rampRate", optRampRate);
  prefs.putInt("cx", centerX);
  prefs.putInt("cy", centerY);
  prefs.putInt("dz", deadzone);
  prefs.end();
}
void savePeerMacToNVS(const uint8_t mac[6]) {
  prefs.begin("joy", false);
  prefs.putBytes("peerMac", mac, 6);
  prefs.end();
}

// ===================== Menu =====================
int menuIndex = 0;
const int MENU_COUNT = 6;
const char* menuName(int i) {
  switch (i) {
    case 0: return "Inv Spd";
    case 1: return "Inv Str";
    case 2: return "Curve";
    case 3: return "Speed";
    case 4: return "Turn Mod";
    case 5: return "Ramp";
    default: return "";
  }
}
void menuToggle(int i) {
  switch (i) {
    case 0: optInvSpeed = !optInvSpeed; break;
    case 1: optInvSteer = !optInvSteer; break;
    case 2: optLog      = !optLog;      break;
    case 3: optSpeedMode = (optSpeedMode + 1) % 3; break;
    case 4: optTurnMod = (optTurnMod + 1) % 4; break;
    case 5: optRampRate = (optRampRate + 1) % 4; break;
  }
  saveSettings();
}
void drawMenu() {
  screen.clear();
  int start = menuIndex - 1;
  if (start < 0) start = 0;
  if (start + 3 > MENU_COUNT) start = max(0, MENU_COUNT - 3);
  for (int row = 0; row < 3; ++row) {
    int idx = start + row;
    if (idx >= MENU_COUNT) break;
    int y = 14 + row * 12;
    screen.setCursor(-4, y);
    screen.print(idx == menuIndex ? "~ " : "  ");
    screen.setCursor(5, y);
    screen.print(menuName(idx));
    screen.setCursor(55, y);
    switch (idx) {
      case 0: screen.print(optInvSpeed ? "X" : "O"); break;
      case 1: screen.print(optInvSteer ? "X" : "O"); break;
      case 2: screen.print(optLog ? "Log" : "Lin"); break;
      case 3:
        screen.print(optSpeedMode == 0 ? "High" :
                     optSpeedMode == 1 ? "Med"  : "Low");
        break;
      case 4:
        screen.print(optTurnMod == 0 ? "Off" :
                     optTurnMod == 1 ? "1/2" :
                     optTurnMod == 2 ? "1/4" : "Dyn");
        break;
      case 5:
        screen.print(optRampRate == 0 ? "Off" :
                     optRampRate == 1 ? "Hi" :
                     optRampRate == 2 ? "Med" : "Low");
        break;
    }
  }
  screen.update();
}

// ===================== SELECT =====================
void drawSelect() {
  screen.clear();
  int start = selectIdx - 1;
  if (start < 0) start = 0;
  if (start + 3 > (int)NUM_BOARDS) start = max(0, (int)NUM_BOARDS - 3);
  for (int row = 0; row < 3; ++row) {
    int idx = start + row;
    if (idx >= (int)NUM_BOARDS) break;
    int y = 14 + row * 12;
    screen.setCursor(-4, y);
    screen.print(idx == selectIdx ? "~ " : "  ");
    screen.setCursor(5, y);
    screen.print(BOARDS[idx].name);
  }
  screen.update();
}

// ===================== Setup =====================
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POT_X_PIN, INPUT);
  pinMode(POT_Y_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  screen.begin();
  screen.setFont(u8g2_font_6x10_tr);

  screen.clear();
  screen.setCursor(0, 16); screen.print("ESP-NOW Joystick");
  screen.setCursor(0, 28); screen.print("MAC: "); screen.print(WiFi.macAddress().c_str());
  screen.update();
  delay(600);

  loadSettings();

  // Fast calibration
  const uint32_t T = 1000, t0 = millis();
  long sx = 0, sy = 0; int n = 0;
  while (millis() - t0 < T) {
    sx += analogRead(POT_X_PIN);
    sy += analogRead(POT_Y_PIN);
    n++;
    delay(2);
  }
  centerX = sx / max(1, n);
  centerY = sy / max(1, n);
  saveSettings();

  initEspNow();

  screen.clear();
  screen.setCursor(0, 14); screen.print("Speed: 0");
  screen.setCursor(0, 26); screen.print("Steer: 0");
  screen.setCursor(0, 38); screen.print("Mode: High");
  screen.update();
}

// ===================== Loop =====================
void loop() {
  const uint32_t now = millis();
  int rawX = analogRead(POT_X_PIN);
  int rawY = analogRead(POT_Y_PIN);

  bool btn = buttonPressed();

  // ---------- Button press start ----------
  if (btn && !btnPrev) {
    tBtnDown = now;
    
    // Only reset flags if we're not already in a menu/screen
    if (ui == RUN) {
      menuShown = false;
      selectShown = false;
      buttonReleasedSinceEntry = false;
      justEnteredViaLongPress = false;
      justEnteredViaMacSelect = false;
      justExitedFromMenu = false;
    }
  }

  // ---------- While holding ----------
  if (btn) {
    unsigned held = now - tBtnDown;

    // 8s -> MAC select (from any state)
    if (held >= LONG_PRESS_SELECT_MS) {
      ui = SELECT;
      selectIdx = 0;
      drawSelect();
      selectShown = true;
      buttonReleasedSinceEntry = false;
      justEnteredViaMacSelect = true;
    }
    // 5s -> enter settings (only from RUN state, and not if we just exited)
    else if (held >= LONG_PRESS_MENU_MS && ui == RUN && !justExitedFromMenu) {
      ui = MENU;
      drawMenu();
      menuShown = true;
      buttonReleasedSinceEntry = false;
      justEnteredViaLongPress = true;
    }
    // 5s -> exit settings (only from MENU state)
    else if (held >= LONG_PRESS_MENU_MS && ui == MENU && buttonReleasedSinceEntry) {
      ui = RUN;
      menuShown = false;
      justExitedFromMenu = true;
      screen.clear();
      screen.setCursor(0, 14); screen.print("Speed: 0");
      screen.setCursor(0, 26); screen.print("Steer: 0");
      const char* modeStr = (optSpeedMode==0?"High":optSpeedMode==1?"Med":"Low");
      screen.setCursor(0, 38); screen.print("Mode: "); screen.print(modeStr);
      screen.update();
    }
  }

  // ---------- On release ----------
  if (!btn && btnPrev) {
    unsigned held = now - tBtnDown;

    if (ui == MENU && menuShown) {
      if (justEnteredViaLongPress && held >= LONG_PRESS_MENU_MS) {
        justEnteredViaLongPress = false;
        buttonReleasedSinceEntry = true;
      } else {
        menuToggle(menuIndex);
        drawMenu();
        justEnteredViaLongPress = false; // Clear flag on any action
      }
      buttonReleasedSinceEntry = true;
    }
    else if (ui == SELECT && selectShown) {
      if (justEnteredViaMacSelect) {
        justEnteredViaMacSelect = false;
        buttonReleasedSinceEntry = true;
      } else {
        memcpy(PEER_MAC, BOARDS[selectIdx].mac, 6);
        savePeerMacToNVS(PEER_MAC);
        screen.clear();
        screen.setCursor(0, 20);
        screen.print("Saved. Rebooting");
        screen.update();
        delay(600);
        ESP.restart();
      }
    }
    
    // Clear the exit flag on any button release
    if (justExitedFromMenu) {
      justExitedFromMenu = false;
    }
  }
  btnPrev = btn;

  // ---------- SELECT MODE ----------
  if (ui == SELECT) {
    float xPct = adcToPercent(rawX, centerX);
    if (!xMoved && (xPct > 70 || xPct < -70)) {
      if (xPct > 70) selectIdx = (selectIdx - 1 + (int)NUM_BOARDS) % (int)NUM_BOARDS;
      if (xPct < -70) selectIdx = (selectIdx + 1) % (int)NUM_BOARDS;
      drawSelect(); xMoved = true;
    } else if (xMoved && abs(xPct) < 40) xMoved = false;
    return;
  }

  // ---------- MENU MODE ----------
  if (ui == MENU) {
    float xPct = adcToPercent(rawX, centerX);
    if (!xMoved && (xPct > 70 || xPct < -70)) {
      if (xPct > 70) menuIndex = (menuIndex + 1) % MENU_COUNT;
      if (xPct < -70) menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
      drawMenu(); xMoved = true;
    } else if (xMoved && abs(xPct) < 40) xMoved = false;
    return;
  }

  // ---------- RUN MODE ----------
  float xPct = adcToPercent(rawX, centerX);
  float yPct = adcToPercent(rawY, centerY);
  if (optLog) { xPct = curveLog(xPct); yPct = curveLog(yPct); }

  float limit = SPEED_LIMITS[optSpeedMode];
  xPct = xPct * (limit / 100.0f);
  yPct = yPct * (limit / 100.0f);

  int speed = (int)roundf(clampf(-xPct, -100.0f, 100.0f));
  int steer = (int)roundf(clampf(+yPct, -100.0f, 100.0f));
  
  // Apply turn modifier
  if (optTurnMod == 1) {
    // 1/2 scaling
    steer = (int)roundf(steer * 0.5f);
  } else if (optTurnMod == 2) {
    // 1/4 scaling
    steer = (int)roundf(steer * 0.25f);
  } else if (optTurnMod == 3) {
    // Dynamic scaling based on speed
    float speedFactor = 1.0f - (abs(speed) * 0.5f / 100.0f);
    steer = (int)roundf(steer * speedFactor);
  }
  
  if (optInvSpeed) speed = -speed;
  if (optInvSteer) steer = -steer;

  // Apply ramping if enabled
  if (optRampRate > 0) {
    float maxChange = RAMP_RATES[optRampRate - 1]; // -1 because 0=Off, 1=Hi, 2=Med, 3=Low
    
    // Ramp speed
    if (speed > lastSpeed) {
      speed = (int)min((float)lastSpeed + maxChange, (float)speed);
    } else if (speed < lastSpeed) {
      speed = (int)max((float)lastSpeed - maxChange, (float)speed);
    }
    
    // Ramp steer
    if (steer > lastSteer) {
      steer = (int)min((float)lastSteer + maxChange, (float)steer);
    } else if (steer < lastSteer) {
      steer = (int)max((float)lastSteer - maxChange, (float)steer);
    }
  }

  if (now - tLastSend >= SEND_MS) {
    tLastSend = now;
    sendPacket(speed, steer);
    
    // Update last values for ramping (track actual sent values)
    lastSpeed = speed;
    lastSteer = steer;
  }

  if (now - tLastOLED >= OLED_MS) {
    tLastOLED = now;
    screen.clear();
    char line[24];
    screen.setCursor(0, 14);
    snprintf(line, sizeof(line), "Speed:%5d", speed); screen.print(line);
    screen.setCursor(0, 26);
    snprintf(line, sizeof(line), "Steer:%5d", steer); screen.print(line);
    screen.setCursor(0, 38);
    const char* modeStr = (optSpeedMode==0?"High":optSpeedMode==1?"Med":"Low");
    screen.print("Mode: "); screen.print(modeStr);
    screen.update();
  }
}