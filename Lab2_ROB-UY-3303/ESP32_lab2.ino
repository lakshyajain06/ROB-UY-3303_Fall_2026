// ESP32 firmware for ROB-UY 3303 Lab 2 — Motion Modeling
//
// NO CHANGES from Lab 1 firmware. The ESP32 already reports cumulative encoder
// counts (encA, encB) and the integrated gyro heading in every ACK packet.
// All odometry computation happens in the Python GUI (GUI_lab2.py).
//
// Flash this file as-is using the Arduino IDE or esptool.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// --- AP NETWORK SETTINGS ---
const char *ssid = "Robot_WiFi";
const char *password = "12345678";
unsigned int localPort = 4010;
char packetBuffer[255];
WiFiUDP udp;

// --- MOTOR PINS ---
#define ENA 5
#define IN1 23
#define IN2 32
#define IN3 33
#define IN4 25
#define ENB 26

// --- ENCODER PINS ---
#define ENC_A_CH1 4
#define ENC_A_CH2 19
#define ENC_B_CH1 14
#define ENC_B_CH2 27

// --- LIDAR PINS ---
#define LIDAR_RX_PIN 34
#define LIDAR_TX_PIN 18
#define LIDAR_MOTOR_PIN 13

// --- LIDAR VARIABLES ---
uint8_t packet[5];
int packet_idx = 0;
uint16_t current_scan[72];
uint16_t ready_scan[72];

// --- MPU6050 AND ODOMETRY SETTINGS ---
const int MPU_addr = 0x68;
float currentAngleZ = 0.0;
float last_gyroZ_speed = 0.0;
unsigned long lastImuTime = 0;

// --- DEADZONE SETTINGS ---
const int MIN_PWM = 105;

volatile long encoderA_count = 0;
volatile long encoderB_count = 0;

void IRAM_ATTR updateEncoderA() {
  if (digitalRead(ENC_A_CH2) == HIGH) encoderA_count--;
  else encoderA_count++;
}

void IRAM_ATTR updateEncoderB() {
  if (digitalRead(ENC_B_CH2) == HIGH) encoderB_count++;
  else encoderB_count--;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  updateMotors(0, 0);

  pinMode(ENC_A_CH1, INPUT_PULLUP); pinMode(ENC_A_CH2, INPUT_PULLUP);
  pinMode(ENC_B_CH1, INPUT_PULLUP); pinMode(ENC_B_CH2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_CH1), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_CH1), updateEncoderB, RISING);

  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  WiFi.softAP(ssid, password);
  udp.begin(localPort);
  delay(500);

  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  digitalWrite(LIDAR_MOTOR_PIN, LOW);
  Serial2.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);
  digitalWrite(LIDAR_MOTOR_PIN, HIGH);
  delay(500);
  Serial2.write(0xA5);
  Serial2.write(0x20);

  memset(current_scan, 0, sizeof(current_scan));
  memset(ready_scan, 0, sizeof(ready_scan));

  lastImuTime = micros();
}

// Reads the IMU at ~200 Hz and integrates yaw. Telemetry only - nothing here decides when to stop
void updateGyro() {
  unsigned long currentTime = micros();
  float dt = (currentTime - lastImuTime) / 1000000.0;
  if (dt < 0.005) return;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);

  if (Wire.available() == 2) {
    int16_t rawZ = Wire.read() << 8 | Wire.read();
    last_gyroZ_speed = rawZ / 131.0;

    if (abs(last_gyroZ_speed) > 1.0) {
      currentAngleZ += last_gyroZ_speed * dt;
    } else {
      last_gyroZ_speed = 0.0;
    }
  }
  lastImuTime = currentTime;
}

// Drains the LIDAR serial stream and accumulates a full 360°/5° sector scan into ready_scan
void readLidarSerial() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (packet_idx == 0) {
      uint8_t syncBits = b & 0x03;
      if (syncBits == 0x01 || syncBits == 0x02) { packet[0] = b; packet_idx = 1; }
    }
    else if (packet_idx == 1) {
      if (b & 0x01) { packet[1] = b; packet_idx = 2; }
      else { packet_idx = 0; }
    }
    else {
      packet[packet_idx] = b;
      packet_idx++;

      if (packet_idx == 5) {
        bool start_of_scan = (packet[0] & 0x01) == 1;

        if (start_of_scan) {
          memcpy(ready_scan, current_scan, sizeof(current_scan));
          memset(current_scan, 0, sizeof(current_scan));
        }

        float angle = ((packet[2] << 8) | packet[1]) >> 1;
        angle = angle / 64.0;
        float distance = ((packet[4] << 8) | packet[3]) / 4.0;
        int quality = packet[0] >> 2;

        if (distance > 0 && quality > 0) {
          int sector = ((int)angle % 360) / 5;
          if (current_scan[sector] == 0 || distance < current_scan[sector]) {
            current_scan[sector] = (uint16_t)distance;
          }
        }
        packet_idx = 0;
      }
    }
  }
}

// M,enA,enB,pwmA,pwmB -> the only command this firmware understands: raw manual drive
void processCommand() {
  if (packetBuffer[0] == 'M') {
    int enA, enB, pwmA, pwmB;
    if (sscanf(packetBuffer, "M,%d,%d,%d,%d", &enA, &enB, &pwmA, &pwmB) == 4) {
      if (!enA) pwmA = 0;
      if (!enB) pwmB = 0;
      updateMotors(pwmA, pwmB);
    }
  }
}

// Builds and sends the ACK reply: raw encoder counts, gyro rate/heading, LIDAR scan
void sendTelemetry() {
  char replyBuffer[600];

  int offset = snprintf(replyBuffer, sizeof(replyBuffer), "ACK,%ld,%ld,%.2f,%.2f",
           encoderA_count, encoderB_count, last_gyroZ_speed, currentAngleZ);

  for (int i = 0; i < 72; i++) {
    offset += snprintf(replyBuffer + offset, sizeof(replyBuffer) - offset, ",%u", ready_scan[i]);
  }

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(replyBuffer);
  udp.endPacket();
}

void loop() {
  updateGyro();
  readLidarSerial();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;

    processCommand();
    sendTelemetry();
  }
}

void updateMotors(int targetA, int targetB) {
  targetA = -targetA;  // left motor mounted/wired backwards

  if (abs(targetA) < MIN_PWM) targetA = 0;
  if (abs(targetB) < MIN_PWM) targetB = 0;

  if (targetA > 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, targetA); }
  else if (targetA < 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, -targetA); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }

  if (targetB > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, targetB); }
  else if (targetB < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, -targetB); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0); }
}
