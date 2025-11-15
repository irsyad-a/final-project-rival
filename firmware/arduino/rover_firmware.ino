// Rover Firmware - Generic Dual DC Motor (TB6612FNG/L298N)
// Protocol (ASCII, newline-terminated):
//   MOVE FWD D=<cm> V=<0-255>
//   MOVE BACK D=<cm> V=<0-255>
//   TURN LEFT A=<deg> V=<0-255>
//   TURN RIGHT A=<deg> V=<0-255>
//   STOP
//   HEARTBEAT
//   CAL SET ms_per_cm=<int>
//   CAL SET ms_per_deg=<int>
//
// Open-loop control using timing. Tune ms_per_cm and ms_per_deg.

// ============ Pin configuration (adjust to your wiring) ============
// TB6612 example pins (change as needed)
#define PIN_PWMA   5   // PWM A (left)
#define PIN_AIN1   7
#define PIN_AIN2   8
#define PIN_PWMB   6   // PWM B (right)
#define PIN_BIN1   9
#define PIN_BIN2   10
#define PIN_STBY   3   // optional, set HIGH to enable, tie to 5V if unused

// Optional Gripper Servo (set to -1 to disable)
#include <Servo.h>
#define PIN_GRIP_SERVO 11
Servo grip;
int gripOpenDeg  = 30;
int gripCloseDeg = 110;

// ============ Calibration and speeds ============
volatile int ms_per_cm  = 80;  // default, tune in field
volatile int ms_per_deg = 12;  // default, tune in field

// Helpers
void motorStop();
void motorForward(int pwmL, int pwmR);
void motorBackward(int pwmL, int pwmR);
void motorTurnLeft(int pwm);   // left in reverse, right forward
void motorTurnRight(int pwm);
long parseIntAfter(const String &s, const String &key, long fallback);
void toUpperInPlace(String &s);

// Serial buffer
String rx;

void setup()
{
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Servo gripper
  grip.attach(PIN_GRIP_SERVO);
  grip.write(gripOpenDeg);

  motorStop();
  Serial.begin(115200);
  delay(100);
  Serial.println("OK ROVER READY");
}

void loop()
{
  // Read lines
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n')
    {
      String line = rx;
      rx = "";
      line.trim();
      if (line.length() == 0) continue;
      handleCommand(line);
    }
    else
    {
      if (rx.length() < 120) rx += c;
    }
  }
}

void handleCommand(String line)
{
  String u = line;
  toUpperInPlace(u);

  if (u.startsWith("HEARTBEAT"))
  {
    Serial.println("OK");
    return;
  }
  if (u.startsWith("STOP"))
  {
    motorStop();
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("CAL SET"))
  {
    long v1 = parseIntAfter(u, "MS_PER_CM=", -1);
    if (v1 >= 0) ms_per_cm = (int)v1;
    long v2 = parseIntAfter(u, "MS_PER_DEG=", -1);
    if (v2 >= 0) ms_per_deg = (int)v2;
    Serial.print("OK CAL ");
    Serial.print(ms_per_cm);
    Serial.print(" ");
    Serial.println(ms_per_deg);
    return;
  }
  if (u.startsWith("GRIP OPEN"))
  {
    grip.write(gripOpenDeg);
    delay(250);
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("GRIP CLOSE"))
  {
    grip.write(gripCloseDeg);
    delay(300);
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("MOVE FWD"))
  {
    int cm = (int)parseIntAfter(u, "D=", 10);
    int v  = (int)parseIntAfter(u, "V=", 150);
    v = constrain(v, 0, 255);
    int duration = cm * ms_per_cm;
    motorForward(v, v);
    delay(duration);
    motorStop();
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("MOVE BACK"))
  {
    int cm = (int)parseIntAfter(u, "D=", 10);
    int v  = (int)parseIntAfter(u, "V=", 150);
    v = constrain(v, 0, 255);
    int duration = cm * ms_per_cm;
    motorBackward(v, v);
    delay(duration);
    motorStop();
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("TURN LEFT"))
  {
    int deg = (int)parseIntAfter(u, "A=", 15);
    int v   = (int)parseIntAfter(u, "V=", 130);
    v = constrain(v, 0, 255);
    int duration = deg * ms_per_deg;
    motorTurnLeft(v);
    delay(duration);
    motorStop();
    Serial.println("DONE");
    return;
  }
  if (u.startsWith("TURN RIGHT"))
  {
    int deg = (int)parseIntAfter(u, "A=", 15);
    int v   = (int)parseIntAfter(u, "V=", 130);
    v = constrain(v, 0, 255);
    int duration = deg * ms_per_deg;
    motorTurnRight(v);
    delay(duration);
    motorStop();
    Serial.println("DONE");
    return;
  }

  // Unknown
  Serial.println("ERR 400");
}

// ============ Motor helpers ============
void motorStop()
{
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
}

void motorForward(int pwmL, int pwmR)
{
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, pwmL);
  analogWrite(PIN_PWMB, pwmR);
}

void motorBackward(int pwmL, int pwmR)
{
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMA, pwmL);
  analogWrite(PIN_PWMB, pwmR);
}

void motorTurnLeft(int pwm)
{
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, pwm);
  analogWrite(PIN_PWMB, pwm);
}

void motorTurnRight(int pwm)
{
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMA, pwm);
  analogWrite(PIN_PWMB, pwm);
}

// ============ Utilities ============
void toUpperInPlace(String &s)
{
  for (size_t i = 0; i < s.length(); ++i)
  {
    char c = s[i];
    if (c >= 'a' && c <= 'z') s.setCharAt(i, c - 32);
  }
}

long parseIntAfter(const String &s, const String &key, long fallback)
{
  int idx = s.indexOf(key);
  if (idx < 0) return fallback;
  idx += key.length();
  bool neg = false;
  long val = 0;
  bool found = false;
  for (int i = idx; i < s.length(); ++i)
  {
    char c = s[i];
    if (!found)
    {
      if (c == '-') { neg = true; found = true; }
      else if (c >= '0' && c <= '9') { val = (c - '0'); found = true; }
      else if (c == ' ') continue;
      else break;
    }
    else
    {
      if (c >= '0' && c <= '9') val = val * 10 + (c - '0');
      else break;
    }
  }
  if (!found) return fallback;
  if (neg) val = -val;
  return val;
}


