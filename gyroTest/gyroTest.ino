#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t gyroX, gyroRate;
float gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;

void setup() {  
  mpu.initialize();
  mpu.setXAccelOffset(-2757);
  mpu.setYAccelOffset(588);
  mpu.setZAccelOffset(1645);
  mpu.setXGyroOffset(-12);
  mpu.setYGyroOffset(42);
  mpu.setZGyroOffset(93);
  Serial.begin(115200);
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  
  Serial.println(gyroAngle);
}
