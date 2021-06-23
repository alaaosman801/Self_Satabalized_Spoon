#include"I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include"MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include"Wire.h"
#endif
#include<Wire.h>
#include<Servo.h>
#include <SoftwareSerial.h>
// declear needed variables
//MPU6050 mpu;
const int mpu_add = 0x68;   // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime,pitched;
int c = 0;

float compCoeff = 0.98;
// define the 2 servo motors
Servo servo1;
Servo servo2;

// define vibration module and meanfilter algorithm variables
char flag;
double vib,meanvib;
int cnt;
SoftwareSerial BTSerial(4,3); // RX, TX


void setup_mpu_6050_registers();
void read_mpu_6050_data();
void complementaryFilter();
void debug_mpu();
void manage_servo();
void calculate_IMU_error();

void setup() {
  // put your setup code here, to run once:
 BTSerial.begin(9600);
 pinMode(A0,INPUT);
Wire.begin();
  setup_mpu_6050_registers();
   // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
   // begin serial communication 
   Serial.begin(115200);
   servo1.attach(10);
   servo2.attach(11);
   servo1.write(0);
   servo2.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  BTSerial.listen();
  vib = analogRead(A0);
  cnt++;
  meanvib +=vib;
  // Get Yaw, Pitch and Roll values
read_mpu_6050_data();

// complementery filter
complementaryFilter();

//debug_mpu();
// send data
if(BTSerial.isListening()){
     flag = BTSerial.read();
      if(flag == '1'){
        meanvib /= cnt;  
        //BTSerial.write(meanvib);
        BTSerial.println(String(meanvib));
        Serial.println(String(meanvib)); // for debuging
        cnt = 0;
        meanvib = 0;
  }
manage_servo();
delay(1000);
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
    /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  //Start communicating with the MPU-6050
  Wire.beginTransmission(mpu_add); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Start communicating with the MPU-6050
  Wire.beginTransmission(mpu_add); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Start communicating with the MPU-6050
  Wire.beginTransmission(mpu_add);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
  }

    
void read_mpu_6050_data(){
   // === Read accelerometer data === //
Wire.beginTransmission(mpu_add);
Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
// Read 6 registers total, each axis value is stored in 2 registers
Wire.requestFrom(mpu_add,6,true);  
 //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
 
   // === Read gyroscope data === //
   // Previous time is stored before the actual time read
  previousTime = currentTime; 
  // Current time actual time read       
  currentTime = millis();
  // Divide by 1000 to get seconds            
  elapsedTime = (currentTime - previousTime) / 1000; 
  Wire.beginTransmission(mpu_add);
   // Gyro data first register address 0x43
  Wire.write(0x43);
  Wire.endTransmission(false);
  // Read 4 registers total, each axis value is stored in 2 registers
  Wire.requestFrom(mpu_add, 6, true); 
  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; 
  }
void complementaryFilter(){
 // Calculating Roll and Pitch from the accelerometer data
 // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; 
  // AccErrorY ~(-1.58)
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; 
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s,
  //so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // complementary Filter 
  roll = ((compCoeff*gyroAngleX)+((1.0f-compCoeff)*accAngleX));
  pitch = ((compCoeff*gyroAngleY)+((1.0f-compCoeff)*accAngleY));  
  }
 void debug_mpu(){
  // draw accelamoter angles
  Serial.print("AccX: ");
 Serial.println(AccX);
  //Serial.println("/");
  Serial.print("AccY: ");
  Serial.println(AccY);
 // Serial.print("/");
 Serial.print("AccZ: ");
  Serial.println(AccZ);
  // roll, pitch, yaw
  // Print the values on the serial monitor
  Serial.print("Roll: ");
  Serial.println(roll);
  //Serial.println("/");
  Serial.print("Pitch: ");
  Serial.println(pitch);
 // Serial.print("/");
  Serial.print("Yaw: ");
  Serial.println(yaw);

  // draw accelamoter angles
  Serial.print("GyroX: ");
 Serial.println(GyroX);
  //Serial.println("/");
  Serial.print("GyroY: ");
  Serial.println(GyroY);
 // Serial.print("/");
 Serial.print("GyroZ: ");
  Serial.println(GyroZ);
  }
void manage_servo(){
    // make roll go to where it's meamt to go
  if(pitch <-158){
    pitched = abs(pitch + 158);
    pitched = pitched - 158;
    }
    else if(pitch > -165){
      pitched = abs(156 + pitch);
      pitched = -156 - pitched;
      }
    // upward movement 
    if(pitched <-240){
      pitched = -240;
      }
      
     //Servo commands, roll/pitch + nr
     //where nr is compensation for mountingto start horizontally
     servo1.write((roll+120));
     servo2.write((pitched+340));
  }

void calculate_IMU_error(){
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(mpu_add);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_add, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(mpu_add);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_add, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: "); Serial.println(GyroErrorZ);
  }
