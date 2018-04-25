#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define leftMotorPWMPin   6
#define leftMotorDirPin   11
#define rightMotorPWMPin  5
#define rightMotorDirPin  3

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
/*
#define Kp  15
#define Kd  0.13
#define Ki  20
*/

/*
float Kp=  17.0;
float Kd=  0.05;
float Ki=  22.5;
float targetAngle = 6.0;
*/
/*
float Kp=  32.0;
float Kd=  -0.05;
float Ki=  30.5;
float targetAngle = 6.0;
*/

/*
float Kp=  36.50;
float Kd=  -0.15;
float Ki=  30.5;
float targetAngle = 3.0;
*/
/*
float Kp=  25.0;
float Kd=  0;
float Ki=  0;
float targetAngle = 3.0;
*/

/*
float Kp=  47.50;
float Kd=  -0.53;
float Ki=  62.50;
float targetAngle = 0.50;
*/


float Kp=  45.0;
float Kd=-0.2  ;
float Ki=  135;
float targetAngle = 1.50;

float motorADutyOffset=1;
float motorBDutyOffset=1;

#define sampleTime  0.005

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
 /* Serial.print(leftMotorSpeed);
  Serial.print(":");Serial.print(-leftMotorSpeed);
  Serial.print(":");
  Serial.print(rightMotorSpeed);
  Serial.print(":");
  Serial.println(-rightMotorSpeed);
  */
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed*motorADutyOffset);
    analogWrite(leftMotorDirPin, 0);
  }
  else {
    analogWrite(leftMotorPWMPin,0);
    analogWrite(leftMotorDirPin,  -leftMotorSpeed*motorADutyOffset);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed*motorBDutyOffset);
    analogWrite(rightMotorDirPin, 0);
  }
  else {
    analogWrite(rightMotorPWMPin,0);
    analogWrite(rightMotorDirPin, -rightMotorSpeed*motorBDutyOffset);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  inputString.reserve(200);
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setXAccelOffset(-2816);
  mpu.setYAccelOffset(562);
  mpu.setZAccelOffset(1633);
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(91);  
  // initialize PID sampling loop
  init_PID();
}
void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  if (stringComplete) {
    if(inputString.equals("angle:H\r\n"))
    {
     targetAngle+=0.5;
    }
    else if(inputString.equals("angle:L\r\n"))
    {
      targetAngle-=0.5;
    }
    else if(inputString.equals("i:H\r\n"))
    {
      Ki+=0.5;
    }    
    else if(inputString.equals("i:L\r\n"))
    {
      Ki-=0.5;      
    }
    else if(inputString.equals("d:H\r\n"))
    {
      Kd+=0.01;
    }
    else if(inputString.equals("d:L\r\n"))
    {
      Kd-=0.01;
    }
    else if(inputString.equals("p:H\r\n"))
    {
      Kp+=0.5;
    }    
    else if(inputString.equals("p:L\r\n"))
    {
      Kp-=0.5;
    }	
	else if(inputString.equals("da:H\r\n"))
    {
      motorADutyOffset+=1;
    }    
    else if(inputString.equals("da:L\r\n"))
    {      
      motorADutyOffset-=1;
    }
	else if(inputString.equals("db:H\r\n"))
    {
      motorBDutyOffset+=1;
    }    
    else if(inputString.equals("db:L\r\n"))
    {
      motorBDutyOffset-=1;
    }
  	
  	else if(inputString.indexOf("angle:")!=-1)
  	{
      targetAngle=(inputString.substring(6)).toFloat();
  	}
    else if(inputString.indexOf("p:")!=-1)
    {
      Kp=(inputString.substring(2)).toFloat();
    }
    else if(inputString.indexOf("d:")!=-1)
    {      
      Kd=(inputString.substring(2)).toFloat();
    }
    else if(inputString.indexOf("i:")!=-1)
    {      
      Ki=(inputString.substring(2)).toFloat();
    }
	else if(inputString.indexOf("dutyA:")!=-1)
	{
		motorADutyOffset=(inputString.substring(6)).toFloat();
	}
	else if(inputString.indexOf("dutyB:")!=-1)
	{
		motorBDutyOffset=(inputString.substring(6)).toFloat();
	}
    else if(inputString.equals("ok:1\r\n"))
    {
      Serial.print(" Kp:");
      Serial.print(Kp);
      Serial.print(" Ki:");
      Serial.print(Ki);
      Serial.print(" Kd:");
      Serial.print(Kd);
      Serial.print(" targetAngle:");
   	  Serial.print(targetAngle);
	  Serial.print(" A offset:");
   	  Serial.print(motorADutyOffset);
	  Serial.print(" B offset:");
   	  Serial.println(motorBDutyOffset);	 
    }
    inputString = "";
    stringComplete = false;
  }
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //Serial.print("error sum:");Serial.println(errorSum);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  //Serial.print("motor power:");Serial.println(motorPower);
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
