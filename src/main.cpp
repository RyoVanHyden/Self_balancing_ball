#include <Arduino.h>
#include <math.h>
#include "controller.h"
#include <Adafruit_VL53L0X.h>
#include <WString.h>
#include <ServoHiwonder.hpp>
#include <SoftwareSerial.h>
#include <ServoHiwonderClass.hpp>
#include <MPU6050.h>
#include <Wire.h>

#define ANGLE_MAX 30
#define ANGLE_MIN -30

#define ROLL_OFFSET -3.3
#define PITCH_OFFSET -2.0

#define MAIN 0
#define TEST 1

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float distance[] = {0.0, 0.0, 0.0, 0.0};

SoftwareSerial serial_motors(2, 3);  // RX, TX
ServoController servoCont(serial_motors);

MPU6050 mpu;
const int MPU = 0x68;
int16_t ax, ay, az, gx, gy, gz;
float accelX, accelY, accelZ, roll, pitch;

//Angle testing ------------
float servo_angles[180];
float measured_angles[180];
int test_index = 0;
float avg_angle = 0.0;
bool done = false;
float last_test_time = 0;
float test_interval = 333;
//--------------------------

int dist_index = 0;

float distance_sampling_time = 15;
float control_sampling_time = 60;

float last_distance_sample, last_control_sample;

float now;

//Control Variables =================================================

float Kc = 2.2327;
float Td = 0.3701;

float distance_INPUT;
float servo_angle_OUTPUT;

float distance_REF = 18.0;
float distance_ERROR = 0.0;

Controller controller("servo angle controller", true, false, true, ANGLE_MAX, ANGLE_MIN, control_sampling_time);

#if MAIN

void setup() {
  
  lox.begin();
  Serial.begin(115200);
  controller.setProportional(Kc);
  controller.setDerivative(Td);

}

void loop() {
  
  now = millis();
  VL53L0X_RangingMeasurementData_t measure;

  //SAMPLE DISTANCE
  if (now - last_distance_sample > distance_sampling_time){
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance[dist_index] = float(measure.RangeMilliMeter) - 25.0;
      Serial.print("Distance (mm): " + String(distance[dist_index])); 
      if (dist_index < 3){
        dist_index++;
      } else if (dist_index == 3){
        dist_index = 0;
      }
    } else {
      Serial.println("TOF sensor out of range ");
    }

    last_distance_sample = now;
  }

  //MAIN CONTROL LOOP
  if (now - last_control_sample > control_sampling_time){

    distance_INPUT = takeDistanceAverage();
    distance_ERROR = distance_REF - distance_INPUT;
    servo_angle_OUTPUT = deg2rad(controller.computeOutput(distance_ERROR));    

    //write servo_angle_OUTPUT on the servo motor (...)
    servoCont.moveWithTime(1, servo_angle_OUTPUT, 60);

    last_control_sample = now;
  }
}

#endif

#if TEST
void setup() {
  lox.begin();
  Serial.begin(115200);
  mpu.initialize();
  Wire.begin();

  for (int i = 0; i<180; i++) {
    servo_angles[i] = i/1.0;
    measured_angles[i] = 0.0;
  }
  test_index = 0;
  avg_angle = 0.0;
  pitch = 0.0;

}

void readIMU(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer data to g-force
  accelX = ax / 16384.0;  // MPU6050 full-scale range ±2g
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;

  // Calculate roll and pitch angles (in degrees)
  roll  = ROLL_OFFSET + atan2(accelY, accelZ) * 180.0 / PI;
  pitch = PITCH_OFFSET + atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  if (roll > 180.0){roll = roll - 360.0;}

  if (abs(roll) < 2.0){roll = 0.0;}
  if (abs(pitch) < 2.0){pitch = 0.0;}

}

void loop() {
    
  now = millis();
  if ((now - last_test_time > test_interval) && (test_index < 180)){

    //Write servo angle on the servo motor and wait for its stabilization
    servoCont.moveWithTime(1, servo_angles[test_index], 60);
    delay(100);

    //Measure pitch angle (take average)
    avg_angle = 0.0;
    for (int i = 0; i<5; i++){
      readIMU();
      avg_angle += pitch;
      if (i!=4) delay(20);
    }
    pitch = avg_angle/5.0;

    //Store measured angle
    measured_angles[test_index] = pitch;
    Serial.println("[i = " + String(test_index) + "]: Servo angle = " + String(servo_angles[test_index]) + "; Measured Angle = " + String(pitch));
    
    test_index++;

    last_test_time = now;
  }

  if ((test_index == 180) && !done){
    done = true;
    for (int i = 0; i<180;i++){
      Serial.println(String(servo_angles[i]) + ", " + String(measured_angles[i]));
    }
  }

}

#endif

float takeDistanceAverage(){
  float dist_sum = 0.0;

  for (int i = 0; i < 4; i++){
    dist_sum = dist_sum + distance[i];
  }

  return dist_sum/4.0;
}

float deg2rad(float deg){
  return deg * PI / 180.0;
}

