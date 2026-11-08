// Stepper tilt for one RPLidar C1 on an Adafruit DC & Stepper Motor HAT.
// The HAT is the PCA9685 + TB6612 board, I2C address 0x60, same as Motor Shield V2.
//
// XIAO ESP32-C5 to the Pi header:
//   D4 SDA -> pin 3, D5 SCL -> pin 5, 3V3 -> pin 1, GND -> pin 6.
// One bipolar stepper on M1 and M2 (port 1). Its supply is the HAT screw terminal.
// Do not power the motor from the XIAO, and do not jumper motor power onto the
// header 5V pin while the XIAO is connected.
//
// Open loop: the pose at reset is angle 0. Level the lidar, then reset.
// ZERO marks the current pose as 0 and does not move.
// The TB6612 has no current limit. kRunScale and kHoldScale cap the PWM so a
// 12 V supply does not hold the coils at full voltage.

#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include <Wire.h>

#include <stdlib.h>
#include <string.h>

static const uint8_t kHatAddr = 0x60;
static const uint16_t kFullStepsPerRev = 200;
static const float kGearRatio = 1.0f;
static const int kStepSign = 1;
static const float kMinDeg = -60.0f;
static const float kMaxDeg = 60.0f;
static const uint32_t kSettleMs = 25;
static const float kSweepDegPerSec = 24.0f;
static const int kMicroSteps = 16;
static const float kRunScale = 0.45f;
static const float kHoldScale = 0.25f;
static_assert(MICROSTEPS == kMicroSteps, "degree math must match the shield microstep table");

// Stepper 1 on the HAT: M1 is PWM 8, M2 is PWM 13.
static const uint8_t kPwmA = 8;
static const uint8_t kAin1 = 10;
static const uint8_t kAin2 = 9;
static const uint8_t kPwmB = 13;
static const uint8_t kBin1 = 12;
static const uint8_t kBin2 = 11;
static const uint8_t kCurve[17] = {0,   25,  50,  74,  98,  120, 141, 162, 180,
                                   197, 212, 225, 236, 244, 250, 253, 255};

// The USB port drops bytes printed before a host opens it, so READY repeats
// once a second until the first command.
static bool gHeardHost = false;
static uint32_t gLastReadyMs = 0;
static float gDegrees = 0.0f;
static int32_t gSteps = 0;
static bool gHatReady = false;
static char gI2cLine[80] = "I2C none";
static uint8_t gPhase = 0;

static Adafruit_MotorShield gShield(kHatAddr);

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

static int32_t degreesToSteps(float degrees) {
  const float microPerDeg =
      (static_cast<float>(kFullStepsPerRev) * static_cast<float>(kMicroSteps) * kGearRatio) / 360.0f;
  return lroundf(degrees * microPerDeg);
}

static void waitUntil(uint32_t dueUs) {
  while (static_cast<int32_t>(dueUs - micros()) > 0) {
    const uint32_t left = dueUs - micros();
    if (left > 2000) {
      delay(1);
    } else {
      delayMicroseconds(left);
      break;
    }
  }
}

static void scanBus() {
  Wire.setTimeOut(5);
  size_t used = 0;
  auto append = [&](const char* text) {
    const size_t n = strlen(text);
    if (used + n + 1 >= sizeof(gI2cLine)) {
      return;
    }
    memcpy(gI2cLine + used, text, n + 1);
    used += n;
  };
  append("I2C");
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      char token[8];
      snprintf(token, sizeof(token), " 0x%02X", addr);
      append(token);
      ++found;
    }
  }
  if (found == 0) {
    append(" none");
  }
}

static void writeCoils(uint8_t phase, float scale) {
  phase = static_cast<uint8_t>(phase % (kMicroSteps * 4));
  uint8_t ocra = 0;
  uint8_t ocrb = 0;
  uint8_t latch = 0;
  if (phase < kMicroSteps) {
    ocra = kCurve[kMicroSteps - phase];
    ocrb = kCurve[phase];
    latch = 0x03;
  } else if (phase < kMicroSteps * 2) {
    ocra = kCurve[phase - kMicroSteps];
    ocrb = kCurve[kMicroSteps * 2 - phase];
    latch = 0x06;
  } else if (phase < kMicroSteps * 3) {
    ocra = kCurve[kMicroSteps * 3 - phase];
    ocrb = kCurve[phase - kMicroSteps * 2];
    latch = 0x0C;
  } else {
    ocra = kCurve[phase - kMicroSteps * 3];
    ocrb = kCurve[kMicroSteps * 4 - phase];
    latch = 0x09;
  }
  const auto duty = [&](uint8_t value) {
    return static_cast<uint16_t>(value * 16.0f * scale);
  };
  gShield.setPWM(kPwmA, duty(ocra));
  gShield.setPWM(kPwmB, duty(ocrb));
  gShield.setPin(kAin2, (latch & 0x1) ? HIGH : LOW);
  gShield.setPin(kBin1, (latch & 0x2) ? HIGH : LOW);
  gShield.setPin(kAin1, (latch & 0x4) ? HIGH : LOW);
  gShield.setPin(kBin2, (latch & 0x8) ? HIGH : LOW);
}

static void microstep(uint8_t dir) {
  if (dir == FORWARD) {
    ++gPhase;
  } else {
    --gPhase;
  }
  gPhase = static_cast<uint8_t>((gPhase + kMicroSteps * 4) % (kMicroSteps * 4));
  writeCoils(gPhase, kRunScale);
}

static bool startHat() {
  if (gHatReady) {
    return true;
  }
  if (!gShield.begin(1600, &Wire)) {
    return false;
  }
  writeCoils(gPhase, kHoldScale);
  gHatReady = true;
  return true;
}

static void rampTo(float target) {
  const int32_t targetSteps = degreesToSteps(target);
  const int32_t delta = targetSteps - gSteps;
  if (delta == 0) {
    gDegrees = target;
    gSteps = targetSteps;
    writeCoils(gPhase, kHoldScale);
    return;
  }
  const int32_t count = delta > 0 ? delta : -delta;
  const uint8_t dir = (delta * kStepSign) > 0 ? FORWARD : BACKWARD;
  const float distance = target > gDegrees ? target - gDegrees : gDegrees - target;
  uint32_t durationUs = static_cast<uint32_t>((distance / kSweepDegPerSec) * 1000000.0f);
  if (durationUs < static_cast<uint32_t>(count) * 1500u) {
    durationUs = static_cast<uint32_t>(count) * 1500u;
  }
  const uint32_t started = micros();
  for (int32_t i = 1; i <= count; ++i) {
    const float u = static_cast<float>(i) / static_cast<float>(count);
    const float eased = u * u * (3.0f - 2.0f * u);
    waitUntil(started + static_cast<uint32_t>(eased * static_cast<float>(durationUs)));
    microstep(dir);
  }
  writeCoils(gPhase, kHoldScale);
  gSteps = targetSteps;
  gDegrees = target;
}

static void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (strcmp(line, "ZERO") == 0) {
    gDegrees = 0.0f;
    gSteps = 0;
    Serial.println("AT 0.0");
    return;
  }
  if (strncmp(line, "GOTO ", 5) != 0) {
    return;
  }
  float degrees = 0.0f;
  if (!parseDegrees(line + 5, degrees) || degrees < kMinDeg || degrees > kMaxDeg) {
    Serial.println("ERR limit");
    return;
  }
  if (!startHat()) {
    Serial.println("ERR i2c");
    return;
  }
  rampTo(degrees);
  delay(kSettleMs);
  Serial.print("AT ");
  Serial.println(degrees, 1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);
  scanBus();
  startHat();
  Serial.println("READY");
  Serial.println(gI2cLine);
  gLastReadyMs = millis();
}

void loop() {
  static char line[64];
  static size_t used = 0;
  if (!gHeardHost && millis() - gLastReadyMs >= 1000) {
    Serial.println("READY");
    Serial.println(gI2cLine);
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
