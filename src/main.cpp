// MPU-6050 -> ESP32 -> Wi-Fi -> HTTPS POST -> Grafana Cloud -> dashboard
//
// Reads the sensor at 50 Hz and runs the tilt-based sit/empty detector
// (lib/occupancy). Every 5 s, and right away when the state changes, it sends
// one line in InfluxDB line protocol, for example:
//   chair,dev=chair01 state=1,accel=9.812,wiggle=0.0123,rssi=-61
// In Grafana these show up as chair_state, chair_accel, chair_wiggle and chair_rssi.
//   state  = 0 empty, 1 sitting
//   accel  = acceleration magnitude in m/s^2 (about 9.8 when still)
//   wiggle = standard deviation of accel over the last 5 s (how much it's moving)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include "occupancy.h"
#include "../lib/OccupancyTimer.h"
#include "secrets.h"

const int I2C_SDA = 21, I2C_SCL = 22;
const uint8_t MPU_ADDR = 0x68;
const uint32_t SAMPLE_MS = 20;    // read the sensor 50 times a second
const uint32_t SEND_MS = 5000;    // send to Grafana every 5 seconds
const float ACCEL_SCALE = 9.80665f / 16384.0f;  // +/-2 g range -> m/s^2
const float GYRO_SCALE = 1.0f / 131.0f;         // +/-250 deg/s range -> deg/s

OccupancyDetector occupancy;
OccupancyTimer occTimer;

// Running sums for the current 5 second window
double sum = 0, sumSq = 0;
uint32_t count = 0;
float lastAccel = 0;

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads accel (m/s^2) into a[0..2] and gyro (deg/s) into g[0..2].
bool readMPU(float a[3], float g[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H; 14 bytes: accel x,y,z, temperature, gyro x,y,z
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  for (int i = 0; i < 3; i++) {
    a[i] = raw[i] * ACCEL_SCALE;
    g[i] = raw[i + 4] * GYRO_SCALE;  // raw[3] is temperature, skipped
  }
  return true;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConnected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWi-Fi failed, will retry next send");
  }
}

void sendToGrafana(int state, float accel, float wiggle) {
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  // The connection is still encrypted, but the server isn't checked yet.
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, GRAFANA_URL)) {
    Serial.println("http.begin failed");
    return;
  }
  http.addHeader("Content-Type", "text/plain");
  http.addHeader("Authorization", String("Bearer ") + GRAFANA_USER + ":" + GRAFANA_TOKEN);

  char body[128];
  snprintf(body, sizeof(body), "chair,dev=%s state=%d,accel=%.3f,wiggle=%.4f,rssi=%d",
           DEVICE_ID, state, accel, wiggle, WiFi.RSSI());

  int code = http.POST((uint8_t *)body, strlen(body));
  Serial.printf("POST %s -> %d\n", body, code);
  if (code != 204) {
    Serial.println("  response: " + http.getString());
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU-6050 not found at 0x68: check SDA/SCL, 3V3 and GND");
  }
  writeReg(0x6B, 0x00);  // wake up
  writeReg(0x1A, 0x03);  // low-pass filter ~44 Hz
  writeReg(0x1B, 0x00);  // gyro +/-250 deg/s
  writeReg(0x1C, 0x00);  // accel +/-2 g
  connectWiFi();

  occTimer.Init(&occupancy);
}

void loop() {
  static uint32_t lastSample = 0, lastSend = 0;
  uint32_t now = millis();
  bool changed = false;

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    float a[3], g[3];
    if (readMPU(a, g)) {
      changed = occupancy.update(a[0], a[1], a[2], g[0], g[1], g[2]);
      if (changed) {
        Serial.printf("-> %s (tilt diff %.3f)\n", occupancy.occupied() ? "SITTING" : "EMPTY",
                      occupancy.lastDistance());
      }
      float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      lastAccel = mag;
      sum += mag;
      sumSq += (double)mag * mag;
      count++;
    }
  }

  if ((changed || now - lastSend >= SEND_MS) && count > 1) {
    lastSend = now;
    double mean = sum / count;
    double var = sumSq / count - mean * mean;
    float wiggle = var > 0 ? sqrt(var) : 0;
    sendToGrafana(occupancy.state(), lastAccel, wiggle);
    sum = sumSq = 0;
    count = 0;
  }
}
