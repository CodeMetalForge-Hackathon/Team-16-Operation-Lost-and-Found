// Hobby-servo tilt for one RPLidar C1.
// Wiring: servo 5V to the XIAO 5V pin, brown to GND, signal to D0.
// A small servo can take 5V from USB. Do not power the servo from 3V3.
// Commands arrive on the USB serial port. This sketch does not talk to the lidar.
//
// Pulse width is center_us + (degrees + zero_offset_deg) * us_per_deg.
// Each GOTO eases from the current angle to the target, then waits settle_ms.
// The reply is the commanded angle. There is no encoder.

#include <Arduino.h>
#include <stdlib.h>

static const int kServoPin = D0;
static const int kCenterUs = 1500;
static const float kUsPerDeg = 10.0f;
static const float kZeroOffsetDeg = 0.0f;
static const float kMinDeg = -30.0f;
static const float kMaxDeg = 30.0f;
static const int kMinPulseUs = 600;
static const int kMaxPulseUs = 2400;
static const uint32_t kSettleMs = 400;
static const float kSweepDegPerSec = 8.0f;
static const uint32_t kRampStepMs = 20;
static const int kPwmHz = 50;
static const int kPwmBits = 14;

// The USB port drops bytes printed before a host opens it, so READY repeats
// once a second until the first command.
static bool gHeardHost = false;
static uint32_t gLastReadyMs = 0;
static float gDegrees = 0.0f;

static bool parseDegrees(const char* token, float& degrees) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char* end = nullptr;
  degrees = strtof(token, &end);
  if (end == token) {
    return false;
  }
  while (*end == ' ' || *end == '\r' || *end == '\t') {
    ++end;
  }
  return *end == '\0';
}

static void writeDegrees(float degrees) {
  const float pulse = kCenterUs + (degrees + kZeroOffsetDeg) * kUsPerDeg;
  float clamped = pulse;
  if (clamped < kMinPulseUs) {
    clamped = kMinPulseUs;
  }
  if (clamped > kMaxPulseUs) {
    clamped = kMaxPulseUs;
  }
  const uint32_t fullScale = 1u << kPwmBits;
  uint32_t duty = static_cast<uint32_t>((clamped / 20000.0f) * fullScale);
  if (duty >= fullScale) {
    duty = fullScale - 1;
  }
  ledcWrite(kServoPin, duty);
}

static void rampTo(float target) {
  const float start = gDegrees;
  const float travel = target - start;
  const float distance = travel < 0.0f ? -travel : travel;
  const uint32_t durationMs = static_cast<uint32_t>((distance / kSweepDegPerSec) * 1000.0f);
  const uint32_t started = millis();
  while (millis() - started < durationMs) {
    const float u = static_cast<float>(millis() - started) / static_cast<float>(durationMs);
    const float eased = u * u * (3.0f - 2.0f * u);
    gDegrees = start + travel * eased;
    writeDegrees(gDegrees);
    delay(kRampStepMs);
  }
  gDegrees = target;
  writeDegrees(gDegrees);
}

static void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (strncmp(line, "GOTO ", 5) != 0) {
    return;
  }
  float degrees = 0.0f;
  if (!parseDegrees(line + 5, degrees) || degrees < kMinDeg || degrees > kMaxDeg) {
    Serial.println("ERR limit");
    return;
  }
  rampTo(degrees);
  delay(kSettleMs);
  Serial.print("AT ");
  Serial.println(degrees, 1);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(kServoPin, kPwmHz, kPwmBits);
  writeDegrees(0.0f);
  Serial.println("READY");
  gLastReadyMs = millis();
}

void loop() {
  static char line[64];
  static size_t used = 0;
  if (!gHeardHost && millis() - gLastReadyMs >= 1000) {
    Serial.println("READY");
    gLastReadyMs = millis();
  }
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (used > 0) {
        gHeardHost = true;
        line[used] = '\0';
        handleLine(line);
        used = 0;
      }
      continue;
    }
    if (used + 1 < sizeof(line)) {
      line[used++] = ch;
    } else {
      used = 0;
    }
  }
}
