/*
 * 2 - MC2: 6-Legged Spider Robot (inside the cave)
 * Board   : Arduino Nano (ATmega328P, 5 V logic)
 * Library : "Adafruit PWM Servo Driver Library"
 *
 * WIRING
 *   PCA9685 SDA  -> A4               PCA9685 SCL -> A5
 *   PCA9685 VCC  -> 5V               PCA9685 GND (logic) -> GND
 *   PCA9685 V+ / GND screw terminal -> separate 5-6 V servo supply (+ / -), GND also tied to Nano GND
 *   HC-05 TXD    -> D10              HC-05 RXD <- D11 through a divider:
 *                                       D11 -[1k]-+-[2k]- GND, middle point -> RXD
 *   HC-05 VCC    -> 5V               HC-05 GND -> GND
 *   IR obstacle OUT -> D2            IR VCC -> 5V, GND -> GND
 *   MQ-4 A7      -> A7 (direct, no divider)   MQ-4 VCC -> 5V, GND -> GND
 *   ALL GROUNDS COMMON (servo supply, PCA9685, Nano, sensors, HC-05)
 *   Power the Nano from a 5 V USB power bank (separate from the servo supply).
 *
 * LEG NUMBERING / PCA9685 CHANNELS  (hip = ch 2*leg, lift = ch 2*leg+1)
 *   0 = Right Front   1 = Right Middle   2 = Right Rear
 *   3 = Left  Front   4 = Left  Middle   5 = Left  Rear
 *
 * PROTOCOL
 *   MC1 -> MC2 (1 character): F forward, B backward, L turn left, R turn right,
 *                             X stop, S sit/stand toggle
 *   MC2 -> MC1 (text line, 4x per second):
 *                             G:<ppm>,O:<0/1>,D:<0/1>,S:<0/1>,W:<0/1>\n
 *                             G=methane ppm, O=obstacle, D=danger, S=sitting, W=sensor warming up
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// ===================== SETTINGS =====================
#define DEBUG_SERIAL        1      // 1 = print to USB Serial Monitor (115200)
#define BT_BAUD             9600   // must match the HC-05 data baud rate

#define DANGER_PPM          1000   // alarm level
#define DANGER_CLEAR_PPM    800    // alarm clears below this (prevents flicker)
#define WARMUP_MS           120000UL  // MQ-4 warm-up (2 min). Use 10000 only for bench tests.
#define CAL_SAMPLES         30     // clean-air samples (about 7.5 s) used to set R0

#define OBSTACLE_ACTIVE_LEVEL LOW  // most IR obstacle modules (FC-51) output LOW when blocked
#define AUTO_STOP_ON_OBSTACLE 0    // 0 = only notify (as in the proposal), 1 = also block forward

#define COMMAND_TIMEOUT_MS  600    // no command for this long -> stop walking (link-lost safety)
#define STEP_MS             160    // time per gait phase; lower = faster walking

// ----- Gait geometry (TUNE THESE during calibration) -----
#define HIP_SWING   25   // degrees the hip swings forward/back from center
#define LIFT_UP     30   // degrees the foot is raised during a step
#define SIT_FOLD    55   // degrees the legs fold when sitting (body lowers)

const int    HIP_CENTER[6] = { 90, 90, 90, 90, 90, 90 };   // hip angle that points the leg straight out
const int    LIFT_STAND[6] = { 90, 90, 90, 90, 90, 90 };   // lift angle when standing, foot on ground
// +1 / -1 fixes mirrored mounting. Flip a sign if that leg moves the wrong way.
const int8_t HIP_DIR[6]    = { +1, +1, +1, -1, -1, -1 };   // +1: larger angle swings the foot forward
const int8_t LIFT_DIR[6]   = { +1, +1, +1, -1, -1, -1 };   // +1: larger angle raises the foot

// MQ-4 load resistor on the module (only used to show Rs; ppm does not depend on it
// because the sensor is calibrated against its own clean-air reading)
const float RL_OHMS         = 10000.0f;
const float CLEAN_AIR_RATIO = 4.4f;      // MQ-4 datasheet: Rs/R0 in clean air

// ===================== PINS =====================
#define BT_RX_PIN    10   // <- HC-05 TXD
#define BT_TX_PIN    11   // -> HC-05 RXD (through the 1k/2k divider)
#define IR_OBST_PIN  2
#define MQ4_PIN      A7
#define ALERT_LED    LED_BUILTIN   // lights while the methane danger alarm is active

// ===================== INTERNALS =====================
#define LEGS 6
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

#if DEBUG_SERIAL
  #define DBG(x)   Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)   do {} while (0)
  #define DBGLN(x) do {} while (0)
#endif

SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN);
Adafruit_PWMServoDriver pwm(0x40);

enum Move { MV_STOP, MV_FWD, MV_BACK, MV_LEFT, MV_RIGHT };
Move moveCmd    = MV_STOP;   // what the operator wants right now
Move activeMove = MV_STOP;   // direction the gait is currently executing
bool sitting    = true;      // boots sitting; operator taps S to stand
bool walking    = false;
uint8_t gaitPhase = 5;
unsigned long nextStepAt = 0, lastCmdAt = 0, lastToggleAt = 0;

// MQ-4 state
unsigned long bootAt = 0;
bool  calibrated = false;
float calSum = 0, R0 = 10000.0f;
uint8_t calCount = 0;
int   ppm = 0;
bool  danger = false, warming = true, obstacle = false;

uint8_t lastAngle[12];       // skip I2C writes when a servo is already at the target

// Tripod gait: group A = legs 0,2,4  group B = legs 1,3,5
// Each phase: { hipPos A, lifted A, hipPos B, lifted B }
// hipPos: +1 = foot ahead (in travel direction), -1 = foot behind
const int8_t PHASE[6][4] = {
  { -1, 1, +1, 0 },   // 0: A lifts
  { +1, 1, -1, 0 },   // 1: A swings forward in the air, B pushes back on the ground
  { +1, 0, -1, 0 },   // 2: A lowers
  { +1, 0, -1, 1 },   // 3: B lifts
  { -1, 0, +1, 1 },   // 4: B swings forward in the air, A pushes back on the ground
  { -1, 0, +1, 0 },   // 5: B lowers
};

// ---------- servo helpers ----------
void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  if (lastAngle[ch] == (uint8_t)angle) return;
  lastAngle[ch] = (uint8_t)angle;
  pwm.writeMicroseconds(ch, map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
}
void setHip(int leg, int a)  { setServo(leg * 2, a); }
void setLift(int leg, int a) { setServo(leg * 2 + 1, a); }

// Legs move one at a time (30 ms apart) to avoid a current spike
void standPose() {
  for (int l = 0; l < LEGS; l++) {
    setHip(l, HIP_CENTER[l]);
    setLift(l, LIFT_STAND[l]);
    delay(30);
  }
}
void sitPose() {
  for (int l = 0; l < LEGS; l++) {
    setHip(l, HIP_CENTER[l]);
    setLift(l, LIFT_STAND[l] + LIFT_DIR[l] * SIT_FOLD);
    delay(30);
  }
}

// ---------- walking ----------
int swingSign(int leg, Move m) {
  bool left = (leg >= 3);
  switch (m) {
    case MV_FWD:   return +1;
    case MV_BACK:  return -1;
    case MV_LEFT:  return left ? -1 : +1;   // right side pushes forward, left side backward
    case MV_RIGHT: return left ? +1 : -1;
    default:       return 0;
  }
}

void runPhase(uint8_t p) {
  for (int l = 0; l < LEGS; l++) {
    bool groupA   = (l == 0 || l == 2 || l == 4);
    int8_t hipPos = groupA ? PHASE[p][0] : PHASE[p][2];
    bool lifted   = groupA ? PHASE[p][1] : PHASE[p][3];
    int s = swingSign(l, activeMove);
    setHip(l,  HIP_CENTER[l] + HIP_DIR[l] * s * hipPos * HIP_SWING);
    setLift(l, LIFT_STAND[l] + (lifted ? LIFT_DIR[l] * LIFT_UP : 0));
  }
}

void gaitTick() {
  if (sitting) return;
  unsigned long now = millis();
  if (now < nextStepAt) return;

  if (moveCmd == MV_STOP) {
    if (!walking) return;
    if (gaitPhase == 0 || gaitPhase == 3) {      // all six feet are down: safe to finish
      standPose();
      walking = false;
      nextStepAt = millis() + STEP_MS;
      return;
    }
  } else if (!walking || moveCmd != activeMove) {
    activeMove = moveCmd;                         // start or change direction
    walking = true;
    gaitPhase = 5;                                // begin from the all-feet-down pose
  }

  runPhase(gaitPhase);
  gaitPhase = (gaitPhase + 1) % 6;
  nextStepAt = millis() + STEP_MS;
}

// ---------- commands from MC1 ----------
void toggleSit() {
  moveCmd = MV_STOP;
  walking = false;
  sitting = !sitting;
  if (sitting) sitPose(); else standPose();
  DBGLN(sitting ? F("SIT") : F("STAND"));
}

void handleCommand(char c) {
  static char lastC = 0;
  switch (c) {
    case 'F': moveCmd = MV_FWD;   break;
    case 'B': moveCmd = MV_BACK;  break;
    case 'L': moveCmd = MV_LEFT;  break;
    case 'R': moveCmd = MV_RIGHT; break;
    case 'X': moveCmd = MV_STOP;  break;
    case 'S':
      if (millis() - lastToggleAt > 600) { lastToggleAt = millis(); toggleSit(); }
      break;
    default: return;                               // ignore noise
  }
  lastCmdAt = millis();
  if (c != lastC) { DBG(F("cmd ")); DBGLN(c); lastC = c; }
}

// ---------- MQ-4 ----------
// The ADC reference and the sensor supply are both the 5 V rail, so the reading is
// a ratio and the exact supply voltage cancels out.
float readRs() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(MQ4_PIN);
  float adc = sum / 8.0f;
  if (adc < 1.0f) adc = 1.0f;
  float rs = RL_OHMS * (1023.0f / adc - 1.0f);
  if (rs < 1.0f) rs = 1.0f;
  return rs;
}

void sensorTick() {
  static unsigned long nextAt = 0;
  unsigned long now = millis();
  if (now < nextAt) return;
  nextAt = now + 250;

  float rs = readRs();
  if (!calibrated) {
    warming = true;
    ppm = 0;
    if (now - bootAt >= WARMUP_MS) {               // warm-up done: learn clean-air R0
      calSum += rs;
      if (++calCount >= CAL_SAMPLES) {
        R0 = calSum / calCount / CLEAN_AIR_RATIO;
        calibrated = true;
        DBG(F("MQ-4 calibrated, R0 = ")); DBG((long)R0); DBGLN(F(" ohm"));
      }
    }
  } else {
    warming = false;
    float p = 1012.7f * pow(rs / R0, -2.786f);     // approximate CH4 curve for the MQ-4
    ppm = (int)constrain(p, 0.0f, 10000.0f);
    if (ppm >= DANGER_PPM) danger = true;
    else if (ppm < DANGER_CLEAR_PPM) danger = false;
  }

  char buf[40];
  snprintf(buf, sizeof(buf), "G:%d,O:%d,D:%d,S:%d,W:%d\n",
           ppm, obstacle ? 1 : 0, danger ? 1 : 0, sitting ? 1 : 0, warming ? 1 : 0);
  bt.print(buf);
#if DEBUG_SERIAL
  Serial.print(buf);
#endif
}

// ---------- main ----------
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(IR_OBST_PIN, INPUT);
  pinMode(ALERT_LED, OUTPUT);
  bt.begin(BT_BAUD);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  delay(10);

  for (uint8_t i = 0; i < 12; i++) lastAngle[i] = 255;   // force the first write to every servo
  sitPose();                       // place the robot on its belly before powering the servos
  bootAt = millis();
  DBG(F("\nMC2 ready (sitting). MQ-4 warming up for "));
  DBG(WARMUP_MS / 1000); DBGLN(F(" s"));
}

void loop() {
  while (bt.available()) handleCommand((char)bt.read());

  if (moveCmd != MV_STOP && millis() - lastCmdAt > COMMAND_TIMEOUT_MS) {
    moveCmd = MV_STOP;             // link lost: stop walking
    DBGLN(F("timeout -> stop"));
  }

  obstacle = (digitalRead(IR_OBST_PIN) == OBSTACLE_ACTIVE_LEVEL);
#if AUTO_STOP_ON_OBSTACLE
  if (obstacle && moveCmd == MV_FWD) moveCmd = MV_STOP;
#endif

  digitalWrite(ALERT_LED, danger ? HIGH : LOW);

  gaitTick();
  sensorTick();
}
