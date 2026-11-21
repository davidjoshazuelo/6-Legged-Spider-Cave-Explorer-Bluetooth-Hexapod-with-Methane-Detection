/*
 * 1 - Servo calibration for MC2 (Arduino Nano + PCA9685, 12 leg servos)
 *
 * Library: "Adafruit PWM Servo Driver Library" (Library Manager)
 * Board  : Arduino Nano (Tools > Processor: ATmega328P, or "Old Bootloader" for many clones)
 *
 * Wiring:
 *   PCA9685 SDA -> Nano A4          PCA9685 SCL -> Nano A5
 *   PCA9685 VCC -> Nano 5V          PCA9685 GND (logic side) -> Nano GND
 *   PCA9685 V+  (screw terminal) -> separate 5-6 V servo supply (+)
 *   PCA9685 GND (screw terminal) -> servo supply (-), and tied to Nano GND
 *   470-1000 uF capacitor across the V+ and GND screw terminals
 *
 * Channel map (same as the main sketch):
 *   leg 0 = Right Front  -> ch 0 (hip)  ch 1 (lift)
 *   leg 1 = Right Middle -> ch 2 (hip)  ch 3 (lift)
 *   leg 2 = Right Rear   -> ch 4 (hip)  ch 5 (lift)
 *   leg 3 = Left Front   -> ch 6 (hip)  ch 7 (lift)
 *   leg 4 = Left Middle  -> ch 8 (hip)  ch 9 (lift)
 *   leg 5 = Left Rear    -> ch 10 (hip) ch 11 (lift)
 *   hip = swings the leg forward/back, lift = raises/lowers the foot
 *
 * Serial Monitor: 115200 baud, line ending = "Newline". Commands:
 *   c            all 12 servos to 90 deg (do this BEFORE attaching the horns)
 *   <ch> <angle> e.g.  "3 120"   moves channel 3 to 120 deg
 *   w <ch>       wiggle one channel (70 -> 110 -> 90)
 *   l <leg>      wiggle hip then lift of one leg
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

#define SERVO_MIN_US 500    // MG90S: about 500 us = 0 deg
#define SERVO_MAX_US 2400   // about 2400 us = 180 deg

char buf[24];

void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.writeMicroseconds(ch, map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
}

void wiggle(uint8_t ch) {
  setServo(ch, 70);  delay(400);
  setServo(ch, 110); delay(400);
  setServo(ch, 90);  delay(300);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  delay(10);
  Serial.println(F("\nServo calibration ready. Commands: c | <ch> <angle> | w <ch> | l <leg>"));
}

void loop() {
  if (!Serial.available()) return;
  int n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[n] = 0;
  while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == ' ')) buf[--n] = 0;   // trim
  if (n == 0) return;

  if (n == 1 && buf[0] == 'c') {
    for (uint8_t ch = 0; ch < 12; ch++) { setServo(ch, 90); delay(80); }  // staggered to limit current
    Serial.println(F("All servos at 90"));
  } else if (buf[0] == 'w') {
    int ch = atoi(buf + 1);
    if (ch >= 0 && ch < 12) {
      wiggle(ch);
      Serial.print(F("Wiggled ch ")); Serial.println(ch);
    }
  } else if (buf[0] == 'l') {
    int leg = atoi(buf + 1);
    if (leg >= 0 && leg < 6) {
      wiggle(leg * 2);       // hip
      wiggle(leg * 2 + 1);   // lift
      Serial.print(F("Wiggled leg ")); Serial.println(leg);
    }
  } else {
    int ch, ang;
    if (sscanf(buf, "%d %d", &ch, &ang) == 2 && ch >= 0 && ch < 12) {
      setServo(ch, ang);
      Serial.print(F("ch ")); Serial.print(ch);
      Serial.print(F(" -> ")); Serial.print(constrain(ang, 0, 180));
      Serial.println(F(" deg"));
    } else {
      Serial.println(F("?"));
    }
  }
}
