#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accY, accZ;
float accAngle;

void setup() {  
  mpu.initialize();
  //-2763  585 1628  -7  33  92
  mpu.setXAccelOffset(-2763);
  mpu.setYAccelOffset(585);
  mpu.setZAccelOffset(1628);
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(92);
  Serial.begin(115200);
}

void loop() {  
  
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.println(accAngle);
}
