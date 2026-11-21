/*
 * 0 - HC-05 AT setup (run on the Arduino Uno, once per HC-05 module)
 *
 * Wiring (Uno -> HC-05):
 *   5V  -> VCC        GND -> GND
 *   D10 <- TXD        (HC-05 3.3V logic into Uno is fine)
 *   D11 -> RXD        through a divider: D11 -[1k]-+-[2k]- GND, middle point -> RXD
 *   KEY/EN pin -> 5V  (or hold the little button on the module while powering up)
 *
 * AT mode = LED blinks slowly (about every 2 s). The AT baud rate is 38400.
 * Serial Monitor: 9600 baud, line ending = "Both NL & CR".
 *
 * Whatever you type goes to the HC-05; whatever it answers shows on screen.
 */
#include <SoftwareSerial.h>

SoftwareSerial bt(10, 11);  // RX, TX

void setup() {
  Serial.begin(9600);
  bt.begin(38400);
  Serial.println(F("HC-05 AT mode ready. Type: AT"));
}

void loop() {
  if (bt.available())     Serial.write(bt.read());
  if (Serial.available()) bt.write(Serial.read());
}
