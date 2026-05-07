#include "BleMouse.h"
#include <Wire.h>
#include <MPU6050.h>

BleMouse bleMouse("ESP32 BLE Mouse", "ESP32", 100);
MPU6050 gyro;

float sensitivity = 2.0;

float axx = 0, ayy = 0, azz = 0;
float roll = 0, pitch = 0;

float roll_est = 0, roll_err = 1;
float pitch_est = 0, pitch_err = 1;
float process_noise = 0.01;
float measurement_noise = 0.5; 

float ax_offset = 0.0; 
float ay_offset = 0.0;
float az_offset = 0.0;  

const int buttonPin = 4;
bool lastButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  delay(3000);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(1000);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(500);

  gyro.initialize();
  delay(500);

  // Manual connection check
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("MPU not found");
    while(1);
  }
  Serial.println("MPU connected!");


  Serial.println("Calibrating... keep device still!");
  int16_t ax, ay, az;
  for(int i = 0; i < 200; i++){ 
    gyro.getAcceleration(&ax, &ay, &az);
    ax_offset += ax / 16384.0;
    ay_offset += ay / 16384.0;
    az_offset += az / 16384.0;
    delay(10);
  }
  ax_offset /= 200;
  ay_offset /= 200;
  az_offset /= 200;

  Serial.println("Calibration done!");
  Serial.print("Offsets -> X: "); Serial.print(ax_offset);
  Serial.print(" Y: "); Serial.print(ay_offset);
  Serial.print(" Z: "); Serial.println(az_offset);

  bleMouse.begin();
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println("BLE started...");
}

float kalmanFilter(float measurement, float &estimate, float &error) {
  error += process_noise;
  float kalman_gain = error / (error + measurement_noise);
  estimate += kalman_gain * (measurement - estimate);
  error *= (1 - kalman_gain);
  return estimate;
}

void loop() {
  // sensitivity via serial monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    sensitivity = input.toFloat();
    Serial.print("Sensitivity: ");
    Serial.println(sensitivity);
  }

  if (!bleMouse.isConnected()) {
    Serial.println("Waiting for BLE...");
    delay(500);
    return;
  }

  int16_t ax, ay, az;
  gyro.getAcceleration(&ax, &ay, &az);

  axx = (ax / 16384.0) - ax_offset;
  ayy = (ay / 16384.0) - ay_offset;
  azz = (az / 16384.0) - az_offset;

  float pitch_raw = atan2(axx, sqrt(ayy*ayy + azz*azz)) * 180 / PI;  // up/down
  float roll_raw  = atan2(ayy, sqrt(axx*axx + azz*azz)) * 180 / PI;  // left/right

  pitch = kalmanFilter(pitch_raw, pitch_est, pitch_err);
  roll  = kalmanFilter(roll_raw,  roll_est,  roll_err);

  float deadzone = 1.5;
  int deltaX = (abs(pitch) > deadzone) ? (int)(pitch * sensitivity) : 0;
  int deltaY = (abs(roll)  > deadzone) ? (int)(roll  * sensitivity) : 0;

  bleMouse.move(deltaX, deltaY);  //

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" | dX: "); Serial.print(deltaX);
  Serial.print(" | dY: "); Serial.println(deltaY);

  bool currentButtonState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    bleMouse.click(MOUSE_LEFT);
    Serial.println("Clicked!");
  }
  lastButtonState = currentButtonState;

  delay(20);
}