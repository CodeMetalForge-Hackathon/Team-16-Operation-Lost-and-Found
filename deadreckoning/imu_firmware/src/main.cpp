// MPU9250 -> serial streamer for the dead-reckoning INS (Seeeduino v4.3 / ATmega328, hardware I2C on A4/A5).
// Prints one line per sample:  I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz     (g, deg/s, microtesla; t in ms)
// Accel/gyro use the MPU9250's own axes (x forward, y left, z up with the chip flat); the magnetometer is
// reported in raw AK8963 axes, which the host converts. Lines starting with '#' are diagnostics.
// If the sensor is wired to A0/A1 instead, build with -DUSE_SOFT_I2C=1 (software I2C, slower).
#include <Arduino.h>

#ifndef SAMPLE_HZ
#define SAMPLE_HZ 100
#endif

#if USE_SOFT_I2C
#include <SoftwareWire.h>
static SoftwareWire BUS(A0, A1);
#else
#include <Wire.h>
#define BUS Wire
#endif

static uint8_t mpuAddr = 0x68;
static float magAdj[3] = {1, 1, 1};

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  BUS.beginTransmission(addr);
  BUS.write(reg);
  BUS.write(val);
  return BUS.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  BUS.beginTransmission(addr);
  BUS.write(reg);
  if (BUS.endTransmission(false) != 0) return false;
  if (BUS.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = BUS.read();
  return true;
}

static void scanBus() {
  Serial.print(F("#   I2C devices: "));
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    BUS.beginTransmission(a);
    if (BUS.endTransmission() == 0) { Serial.print(F("0x")); Serial.print(a, HEX); Serial.print(' '); n++; }
  }
  Serial.println(n ? F("") : F("none answer"));
}

static bool findMpu() {
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    uint8_t who;
    if (readRegs(a, 0x75, &who, 1) && (who == 0x71 || who == 0x73 || who == 0x70)) {
      mpuAddr = a;
      Serial.print(F("# found MPU at 0x")); Serial.print(a, HEX);
      Serial.print(F(" WHO_AM_I=0x")); Serial.println(who, HEX);
      return true;
    }
  }
  return false;
}

static void initImu() {
  writeReg(mpuAddr, 0x6B, 0x80); delay(100);          // reset
  writeReg(mpuAddr, 0x6B, 0x01);                       // clock from the gyro PLL
  writeReg(mpuAddr, 0x1A, 0x03);                       // gyro low-pass ~41 Hz (rejects motor vibration)
  writeReg(mpuAddr, 0x1B, 0x08);                       // gyro +/-500 dps
  writeReg(mpuAddr, 0x1C, 0x08);                       // accel +/-4 g
  writeReg(mpuAddr, 0x1D, 0x03);                       // accel low-pass ~44 Hz
  writeReg(mpuAddr, 0x19, 1000 / SAMPLE_HZ - 1);       // sample rate divider (1 kHz base)
  writeReg(mpuAddr, 0x37, 0x02);                       // bypass: the AK8963 appears on this bus at 0x0C
  delay(10);
  uint8_t who;
  if (!readRegs(0x0C, 0x00, &who, 1) || who != 0x48) {
    Serial.println(F("# AK8963 magnetometer not found (continuing without it)"));
    return;
  }
  writeReg(0x0C, 0x0A, 0x00); delay(10);
  writeReg(0x0C, 0x0A, 0x0F); delay(10);               // fuse ROM: factory sensitivity adjustment
  uint8_t asa[3];
  if (readRegs(0x0C, 0x10, asa, 3)) for (int i = 0; i < 3; i++) magAdj[i] = (asa[i] - 128) / 256.0f + 1.0f;
  writeReg(0x0C, 0x0A, 0x00); delay(10);
  writeReg(0x0C, 0x0A, 0x16); delay(10);               // 16-bit, continuous 100 Hz
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("# MPU9250 streamer starting"));
  BUS.begin();
#if !USE_SOFT_I2C
  BUS.setClock(400000);
#endif
  while (!findMpu()) {
    Serial.println(F("# MPU9250 not found at 0x68/0x69. Check 3.3 V, GND, SDA, SCL and the 3V3/5V switch."));
    scanBus();
    delay(3000);
  }
  initImu();
  Serial.println(F("# streaming"));
}

static int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
static int16_t le16(const uint8_t *p) { return (int16_t)((p[1] << 8) | p[0]); }

void loop() {
  static uint32_t next = 0;
  static float mx = 0, my = 0, mz = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next) < 0) return;
  next = now + 1000 / SAMPLE_HZ;

  uint8_t d[14];
  if (!readRegs(mpuAddr, 0x3B, d, 14)) { Serial.println(F("# IMU read failed")); return; }
  const float ax = be16(d) / 8192.0f, ay = be16(d + 2) / 8192.0f, az = be16(d + 4) / 8192.0f;   // g
  const float gx = be16(d + 8) / 65.5f, gy = be16(d + 10) / 65.5f, gz = be16(d + 12) / 65.5f;    // deg/s

  uint8_t st1, m[7];
  if (readRegs(0x0C, 0x02, &st1, 1) && (st1 & 1) && readRegs(0x0C, 0x03, m, 7) && !(m[6] & 0x08)) {
    mx = le16(m) * 0.15f * magAdj[0]; my = le16(m + 2) * 0.15f * magAdj[1]; mz = le16(m + 4) * 0.15f * magAdj[2];
  }
  Serial.print(F("I,")); Serial.print(now);
  Serial.print(','); Serial.print(ax, 3); Serial.print(','); Serial.print(ay, 3); Serial.print(','); Serial.print(az, 3);
  Serial.print(','); Serial.print(gx, 2); Serial.print(','); Serial.print(gy, 2); Serial.print(','); Serial.print(gz, 2);
  Serial.print(','); Serial.print(mx, 1); Serial.print(','); Serial.print(my, 1); Serial.print(','); Serial.println(mz, 1);
}
