/*
 * 3 - MC1: Handheld Operator Console (outside the cave)
 * Board   : Arduino Uno R3
 * Library : "LiquidCrystal I2C" (Frank de Brabander), via Library Manager
 *
 * WIRING
 *   HC-05 VCC -> 5V, GND -> GND
 *   HC-05 TXD -> D10                       (HC-05 3.3 V into the Uno is fine)
 *   HC-05 RXD <- D11 through a divider:    D11 -[1k]-+-[2k]- GND, middle point -> RXD
 *   Joystick  VRx -> A0, VRy -> A1, +5V -> 5V, GND -> GND
 *   TTP223 touch  OUT -> D2, VCC -> 5V, GND -> GND
 *   LCD 16x2 with I2C backpack: SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND
 *   Buzzer (optional) -> D8 and GND, set ENABLE_BUZZER to 1
 *
 * PROTOCOL  (see the MC2 sketch)
 *   Sends one character 10 times per second: F B L R X, and S once per touch
 *   Receives lines like  G:850,O:0,D:0,S:0,W:0
 *
 * With DEBUG_SERIAL = 1 the Serial Monitor (9600) shows every command sent and
 * every status line received, so you can test the joystick with no robot connected.
 */
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== SETTINGS =====================
#define DEBUG_SERIAL   1
#define BT_BAUD        9600
#define LCD_ADDR       0x27     // try 0x3F if the LCD stays blank
#define JOY_DEADZONE   200      // 0-512; bigger = less sensitive
#define INVERT_Y       0        // set 1 if pushing the stick forward prints B
#define INVERT_X       0        // set 1 if pushing right prints L
#define ENABLE_BUZZER  0
#define LINK_TIMEOUT_MS 2000

// ===================== PINS =====================
#define BT_RX_PIN   10
#define BT_TX_PIN   11
#define JOY_X       A0
#define JOY_Y       A1
#define TOUCH_PIN   2
#define BUZZER_PIN  8

SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// latest status from the robot
int  ppm = 0;
bool obstacle = false, danger = false, sitting = true, warming = true;
unsigned long lastStatusAt = 0;

char line[48];
uint8_t n = 0;

// ---------- LCD helper: only rewrites a row when its text changed ----------
void showRow(uint8_t row, const char *text) {
  static char last[2][17] = { "", "" };
  char padded[17];
  snprintf(padded, sizeof(padded), "%-16s", text);
  if (strcmp(padded, last[row]) != 0) {
    lcd.setCursor(0, row);
    lcd.print(padded);
    strcpy(last[row], padded);
  }
}

// ---------- status from MC2 ----------
void parseStatus() {
  int g, o, d, s, w;
  if (sscanf(line, "G:%d,O:%d,D:%d,S:%d,W:%d", &g, &o, &d, &s, &w) == 5) {
    ppm = g; obstacle = o; danger = d; sitting = s; warming = w;
    lastStatusAt = millis();
#if DEBUG_SERIAL
    Serial.print(F("RX: ")); Serial.println(line);
#endif
  }
}

void readStatus() {
  while (bt.available()) {
    char c = bt.read();
    if (c == '\n') { line[n] = 0; parseStatus(); n = 0; }
    else if (c != '\r') {
      if (n < sizeof(line) - 1) line[n++] = c;
      else n = 0;                                 // garbage: resync
    }
  }
}

// ---------- display ----------
void updateDisplay() {
  static unsigned long nextAt = 0;
  static bool flash = false;
  unsigned long now = millis();
  if (now < nextAt) return;
  nextAt = now + 400;
  flash = !flash;

  char r0[20], r1[20];
  bool linkOk = (lastStatusAt != 0) && (now - lastStatusAt < LINK_TIMEOUT_MS);

  if (!linkOk) {
    showRow(0, "No link to robot");
    showRow(1, "Check HC-05 pair");
  } else if (danger) {
    if (flash) {
      showRow(0, "!! DANGER !!");
      showRow(1, "METHANE DETECTED");        // the warning emoji cannot be shown on a 16x2 LCD
    } else {
      snprintf(r0, sizeof(r0), "CH4:%-5dppm", ppm);
      showRow(0, r0);
      showRow(1, "DRIVE OUT NOW!");
    }
#if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN, flash);
#endif
  } else if (warming) {
    showRow(0, "MQ-4 warming up");
    showRow(1, "Keep in clean air");
  } else {
    snprintf(r0, sizeof(r0), "CH4:%-5dppm", ppm);
    showRow(0, r0);
    snprintf(r1, sizeof(r1), "%-10s %-5s", obstacle ? "OBSTACLE!" : "Path Clear",
             sitting ? "SIT" : "STAND");
    showRow(1, r1);
  }
#if ENABLE_BUZZER
  if (!(linkOk && danger)) digitalWrite(BUZZER_PIN, LOW);
#endif
}

// ---------- controls ----------
char readJoystick() {
  int x = analogRead(JOY_X) - 512;
  int y = analogRead(JOY_Y) - 512;
  if (INVERT_X) x = -x;
  if (INVERT_Y) y = -y;

  if (abs(y) >= abs(x)) {                         // the larger push wins
    if (y >  JOY_DEADZONE) return 'F';
    if (y < -JOY_DEADZONE) return 'B';
  } else {
    if (x >  JOY_DEADZONE) return 'R';
    if (x < -JOY_DEADZONE) return 'L';
  }
  return 'X';
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(9600);
#endif
  bt.begin(BT_BAUD);
  pinMode(TOUCH_PIN, INPUT);
#if ENABLE_BUZZER
  pinMode(BUZZER_PIN, OUTPUT);
#endif
  lcd.init();
  lcd.backlight();
  showRow(0, "Spider Console");
  showRow(1, "Waiting for link");
}

void loop() {
  readStatus();
  updateDisplay();

  // send the drive command 10 times per second
  static unsigned long nextSendAt = 0;
  static char lastCmd = 0;
  unsigned long now = millis();
  if (now >= nextSendAt) {
    nextSendAt = now + 100;
    char c = readJoystick();
    bt.write(c);
#if DEBUG_SERIAL
    if (c != lastCmd) { Serial.print(F("TX: ")); Serial.println(c); }
#endif
    lastCmd = c;
  }

  // touch sensor: one 'S' per touch (rising edge)
  static bool wasTouched = false;
  bool touched = digitalRead(TOUCH_PIN) == HIGH;
  if (touched && !wasTouched) {
    bt.write('S');
#if DEBUG_SERIAL
    Serial.println(F("TX: S (sit/stand)"));
#endif
  }
  wasTouched = touched;
}
