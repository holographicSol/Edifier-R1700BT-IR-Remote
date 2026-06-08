/*
 * Edifier R1700BT IR Remote — Waveshare ESP32-P4-WIFI6
 *
 * Original Android project: https://github.com/wikipedia555/R1700BT
 * Target board:             Waveshare ESP32-P4-WIFI6
 *                           https://www.waveshare.com/esp32-p4-wifi6.htm
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  ⚠  ESP32-P4 Arduino support is early/experimental.                     ║
 * ║  Waveshare recommends ESP-IDF for stable development.                   ║
 * ║  See ESP-IDF note at bottom if IRremoteESP8266 fails to build.          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Libraries required:
 *   IRremoteESP8266  – crankyoldgit  (Library Manager)
 *   Adafruit SSD1306 – Adafruit      (Library Manager, installs GFX too)
 *
 * ── PIN ASSIGNMENTS ───────────────────────────────────────────────────────
 *   GPIO2   IR LED output     (47Ω resistor in series)
 *   GPIO3   BTN_POWER
 *   GPIO4   BTN_MUTE
 *   GPIO5   BTN_VOL_UP
 *   GPIO48  BTN_VOL_DN
 *   GPIO47  BTN_BT
 *   GPIO46  BTN_LINE
 *   GPIO12  BTN_CAL
 *   GPIO33  LED indicator
 *   GPIO51  OLED SCL
 *   GPIO52  OLED SDA
 *
 * Buttons wire active-LOW (pin → button → GND, internal pull-ups enabled).
 * IR LED: GPIO2 → 47Ω → LED(+) → LED(−) → GND  (HW-489 module or bare 940nm LED)
 * OLED:   VCC→3.3V  GND→GND  SCL→GPIO51  SDA→GPIO52  (SSD1306, addr 0x3C)
 *
 * ── SERIAL COMMANDS (115200 baud) ─────────────────────────────────────────
 *   p      Power toggle
 *   m      Mute toggle
 *   u      Volume Up   (updates tracker)
 *   d      Volume Down (updates tracker)
 *   b      Input: Bluetooth
 *   l      Input: Line In (AUX)
 *   c      Calibrate — slams volume to 0, resets tracker
 *   x      Quick test — set volume to 10
 *   0-100  Set absolute volume (requires prior calibration)
 *   ?      Print this help
 *
 * ── VOLUME SYSTEM ─────────────────────────────────────────────────────────
 *   VOL_MAX_STEPS = 100, VOL_STEP_DELAY = 1ms (confirmed working)
 *   Always calibrate ('c') on first use or after power cycling the speaker.
 *   After calibration, numeric entry and setVolume() use fast relative steps,
 *   alternatively, vol +/- can be used to step incrementally without calibration.
 * ─────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ──────────────────────────────────────────────────────────────────
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
#define OLED_SCL      51
#define OLED_SDA      52
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// ── Volume ────────────────────────────────────────────────────────────────
#define VOL_MAX_STEPS  100
#define VOL_STEP_DELAY   1   // ms between IR pulses
uint8_t currentVolume  =  0;

// ── Pins ──────────────────────────────────────────────────────────────────
const uint8_t IR_SEND_PIN = 2;
const uint8_t BTN_POWER   = 3;
const uint8_t BTN_MUTE    = 4;
const uint8_t BTN_VOL_UP  = 5;
const uint8_t BTN_VOL_DN  = 48;
const uint8_t BTN_BT      = 47;
const uint8_t BTN_LINE    = 46;
const uint8_t BTN_CAL     = 12;
const uint8_t LED_PIN     = 33;

// ── NEC IR codes (32-bit, 38 kHz) ─────────────────────────────────────────
const uint64_t NEC_POWER  = 149389439UL;
const uint64_t NEC_MUTE   = 149356799UL;
const uint64_t NEC_VOL_DN = 149369039UL;
const uint64_t NEC_VOL_UP = 149393519UL;
const uint64_t NEC_BT     = 149377199UL;
const uint64_t NEC_LINE   = 149399639UL;
const uint16_t NEC_BIT_NUM = 32;

// ── Debounce ──────────────────────────────────────────────────────────────
const unsigned long DEBOUNCE_MS = 200;

// ── IR sender ─────────────────────────────────────────────────────────────
IRsend irsend(IR_SEND_PIN);

// ── Button struct (no lastPressMs — debounce via lastPress[]) ─────────────
struct Button {
  uint8_t     pin;
  uint64_t    code;
  const char* label;
};

Button buttons[] = {
  { BTN_POWER,  NEC_POWER,  "Power"      },
  { BTN_MUTE,   NEC_MUTE,   "Mute"       },
  { BTN_VOL_UP, NEC_VOL_UP, "Volume Up"  },
  { BTN_VOL_DN, NEC_VOL_DN, "Volume Down"},
  { BTN_BT,     NEC_BT,     "Bluetooth"  },
  { BTN_LINE,   NEC_LINE,   "Line In"    },
};
const uint8_t NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);

// ── ISR flags ─────────────────────────────────────────────────────────────
enum PendingCmd { CMD_NONE, CMD_POWER, CMD_MUTE, CMD_VOLUP, CMD_VOLDN, CMD_BT, CMD_LINE };
volatile PendingCmd    pendingCmd   = CMD_NONE;
volatile bool          calRequested = false;
volatile unsigned long lastCalPress = 0;
volatile unsigned long lastPress[6] = {0,0,0,0,0,0};

// ── OLED / LED async flags ────────────────────────────────────────────────
volatile bool          oledRequested = false;
volatile char          oledLine1[12] = "";
volatile char          oledLine2[12] = "";
volatile unsigned long ledOnTime     = 0;
volatile bool          ledActive     = false;
volatile unsigned long oledShowTime  = 0;

const unsigned long LED_MS  =   50;
const unsigned long OLED_MS = 1500;

// ── OLED helpers ──────────────────────────────────────────────────────────
// Non-blocking — does not delay. Clear is handled by displayTask.
void showOled(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.println(line1);
  if (line2[0]) {
    display.setTextSize(2);
    display.setCursor(0, 40);
    display.println(line2);
  }
  display.display();
}

// ISR-safe flag setter
void IRAM_ATTR requestOled(const char* l1, const char* l2 = "") {
  strncpy((char*)oledLine1, l1, 11); oledLine1[11] = '\0';
  strncpy((char*)oledLine2, l2, 11); oledLine2[11] = '\0';
  oledRequested = true;
}

// ── Display + LED task (core 0, low priority) ─────────────────────────────
void displayTask(void* pvParameters) {
  for (;;) {
    // LED auto-off
    if (ledActive && (millis() - ledOnTime >= LED_MS)) {
      digitalWrite(LED_PIN, LOW);
      ledActive = false;
    }

    // OLED auto-clear
    if (oledShowTime > 0 && (millis() - oledShowTime >= OLED_MS)) {
      oledShowTime = 0;
      display.clearDisplay();
      display.display();
    }

    // OLED update from ISR flag
    if (oledRequested) {
      oledRequested = false;
      showOled((const char*)oledLine1, (const char*)oledLine2);
      oledShowTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ── ISRs — flag-only, zero work ───────────────────────────────────────────
void IRAM_ATTR isr_power() { if (millis()-lastPress[0]>=DEBOUNCE_MS){ lastPress[0]=millis(); pendingCmd=CMD_POWER; requestOled("PWR");         } }
void IRAM_ATTR isr_mute()  { if (millis()-lastPress[1]>=DEBOUNCE_MS){ lastPress[1]=millis(); pendingCmd=CMD_MUTE;  requestOled("MUTE");        } }
void IRAM_ATTR isr_volup() { if (millis()-lastPress[2]>=DEBOUNCE_MS){ lastPress[2]=millis(); pendingCmd=CMD_VOLUP; requestOled("VOL+");        } }
void IRAM_ATTR isr_voldn() { if (millis()-lastPress[3]>=DEBOUNCE_MS){ lastPress[3]=millis(); pendingCmd=CMD_VOLDN; requestOled("VOL-");        } }
void IRAM_ATTR isr_bt()    { if (millis()-lastPress[4]>=DEBOUNCE_MS){ lastPress[4]=millis(); pendingCmd=CMD_BT;    requestOled("BT");          } }
void IRAM_ATTR isr_line()  { if (millis()-lastPress[5]>=DEBOUNCE_MS){ lastPress[5]=millis(); pendingCmd=CMD_LINE;  requestOled("LINE");        } }
void IRAM_ATTR isr_cal()   { if (millis()-lastCalPress >=DEBOUNCE_MS){ lastCalPress=millis(); calRequested=true;   requestOled("CAL","zeroing");} }

// ── IR / volume helpers ───────────────────────────────────────────────────
void sendCommand(uint64_t code, const char* label) {
  irsend.sendNEC(code, NEC_BIT_NUM);
  Serial.print("[IR] Sent: ");
  Serial.println(label);
}

void calVolume() {
  for (int i = 0; i < VOL_MAX_STEPS; i++) {
    irsend.sendNEC(NEC_VOL_DN, NEC_BIT_NUM);
    delay(VOL_STEP_DELAY);
  }
}

void setVolume(uint8_t targetLevel) {
  targetLevel = min(targetLevel, (uint8_t)VOL_MAX_STEPS);
  if (targetLevel == currentVolume) return;
  uint64_t code = (targetLevel > currentVolume) ? NEC_VOL_UP : NEC_VOL_DN;
  int steps = abs((int)targetLevel - (int)currentVolume);
  for (int i = 0; i < steps; i++) {
    irsend.sendNEC(code, NEC_BIT_NUM);
    delay(VOL_STEP_DELAY);
  }
  currentVolume = targetLevel;
  Serial.print("[VOL] Set to ");
  Serial.println(currentVolume);
}

void ledFlash() {
  digitalWrite(LED_PIN, HIGH);
  ledOnTime = millis();
  ledActive = true;
}

void printHelp() {
  Serial.println();
  Serial.println("=== Edifier R1700BT (ESP32-P4-WIFI6) ===");
  Serial.println("  p      – Power toggle");
  Serial.println("  m      – Mute toggle");
  Serial.println("  u      – Volume Up   (tracked)");
  Serial.println("  d      – Volume Down (tracked)");
  Serial.println("  b      – Input: Bluetooth");
  Serial.println("  l      – Input: Line In (AUX)");
  Serial.println("  c      – Calibrate (slam to 0, reset tracker)");
  Serial.println("  x      – Quick test: set volume to 10");
  Serial.println("  0-100  – Set absolute volume (calibrate first)");
  Serial.println("  ?      – This help");
  Serial.println();
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  irsend.begin();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] Init failed — check wiring/address");
  } else {
    showOled("Ready", "R1700BT");
    oledShowTime = millis();
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }
  pinMode(BTN_CAL, INPUT_PULLUP);

  delay(500);

  attachInterrupt(digitalPinToInterrupt(BTN_POWER),  isr_power,  FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_MUTE),   isr_mute,   FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_VOL_UP), isr_volup,  FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_VOL_DN), isr_voldn,  FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_BT),     isr_bt,     FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_LINE),   isr_line,   FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_CAL),    isr_cal,    FALLING);

  xTaskCreatePinnedToCore(
    displayTask,   // function
    "displayTask", // name
    4096,          // stack bytes
    NULL,          // params
    1,             // priority (low)
    NULL,          // handle
    0              // core 0 — loop/IR runs on core 1
  );

  printHelp();
  Serial.println("Ready.");
}

// ── Loop (core 1) ─────────────────────────────────────────────────────────
void loop() {

  // ── Dispatch pending button command ─────────────────────────────────
  if (pendingCmd != CMD_NONE) {
    PendingCmd cmd = pendingCmd;
    pendingCmd = CMD_NONE;
    ledFlash();
    switch (cmd) {
      case CMD_POWER: sendCommand(NEC_POWER,  "Power");       break;
      case CMD_MUTE:  sendCommand(NEC_MUTE,   "Mute");        break;
      case CMD_VOLUP:
        sendCommand(NEC_VOL_UP, "Volume Up");
        if (currentVolume < VOL_MAX_STEPS) currentVolume++;
        break;
      case CMD_VOLDN:
        sendCommand(NEC_VOL_DN, "Volume Down");
        if (currentVolume > 0) currentVolume--;
        break;
      case CMD_BT:    sendCommand(NEC_BT,     "Bluetooth");   break;
      case CMD_LINE:  sendCommand(NEC_LINE,   "Line In");     break;
      default: break;
    }
  }

  // ── Calibration ─────────────────────────────────────────────────────
  if (calRequested) {
    calRequested = false;
    ledFlash();
    calVolume();
    currentVolume = 0;
    Serial.println("[VOL] Calibrated to 0");
    showOled("CAL", "done");
    oledShowTime = millis();
  }

  // ── Serial input ────────────────────────────────────────────────────
  if (Serial.available()) {
    while (Serial.available() && (Serial.peek()=='\n' || Serial.peek()=='\r' || Serial.peek()==' ')) {
      Serial.read();
    }
    if (!Serial.available()) return;

    char c = (char)Serial.read();

    // numeric → absolute volume set
    if (isDigit(c)) {
      String numStr = String(c);
      while (Serial.available() && isDigit(Serial.peek())) numStr += (char)Serial.read();
      while (Serial.available() && (Serial.peek()=='\n' || Serial.peek()=='\r')) Serial.read();
      uint8_t level = (uint8_t)constrain(numStr.toInt(), 0, VOL_MAX_STEPS);
      char buf[12];
      snprintf(buf, sizeof(buf), "VOL %d", level);
      showOled(buf);
      oledShowTime = millis();
      setVolume(level);
      return;
    }

    while (Serial.available() && (Serial.peek()=='\n' || Serial.peek()=='\r')) Serial.read();
    Serial.println("[RCV] 0x" + String((uint8_t)c, HEX) + " '" + String(c) + "'");

    switch (c) {
      case 'p': case 'P':
        sendCommand(NEC_POWER, "Power");
        break;
      case 'm': case 'M':
        sendCommand(NEC_MUTE, "Mute");
        break;
      case 'u': case 'U':
        sendCommand(NEC_VOL_UP, "Volume Up");
        if (currentVolume < VOL_MAX_STEPS) currentVolume++;
        Serial.print("[VOL] ~"); Serial.println(currentVolume);
        break;
      case 'd': case 'D':
        sendCommand(NEC_VOL_DN, "Volume Down");
        if (currentVolume > 0) currentVolume--;
        Serial.print("[VOL] ~"); Serial.println(currentVolume);
        break;
      case 'b': case 'B':
        sendCommand(NEC_BT, "Bluetooth");
        break;
      case 'l': case 'L':
        sendCommand(NEC_LINE, "Line In");
        break;
      case 'c': case 'C':
        showOled("CAL", "zeroing");
        oledShowTime = millis();
        calVolume();
        currentVolume = 0;
        Serial.println("[VOL] Calibrated to 0");
        showOled("CAL", "done");
        oledShowTime = millis();
        break;
      case 'x': case 'X':
        setVolume(10);
        showOled("VOL", "10");
        oledShowTime = millis();
        break;
      case '?':
        printHelp();
        break;
      default:
        Serial.print("[?] Unknown: 0x");
        Serial.println((uint8_t)c, HEX);
        break;
    }
  }
}

/*
 * ── ESP-IDF ALTERNATIVE ───────────────────────────────────────────────────
 * If Arduino-ESP32 P4 support is too unstable, use ESP-IDF with the RMT
 * peripheral directly:
 *   #include "driver/rmt_tx.h"
 *   #include "ir_nec_encoder.h"   // from Espressif examples
 *   // Configure RMT TX on GPIO2, 38kHz carrier, then rmt_transmit()
 *   // Guide: https://docs.waveshare.com/ESP32-P4-WIFI6/Development-Environment-Setup-IDF
 * ─────────────────────────────────────────────────────────────────────────
 */
