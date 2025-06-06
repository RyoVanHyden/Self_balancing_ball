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

#define MAIN 1
#define TEST_ANGLE 0
#define TEST_COMMANDS 0
#define MANUAL 0

#define ANGLE_MAX 0.096
#define ANGLE_MIN -0.096

#define MAX_COMMAND 730.0
#define MIN_COMMAND 250.0

#define ROLL_OFFSET 3.3
#define PITCH_OFFSET 

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
float manual_angle = 0.0;

float command = 250.0;
float command_step = 5.0;
float measured_commands_angle[96];

//--------------------------

int dist_index = 0;

float now;

//Control Variables =================================================

Controller controller("servo angle controller", true, false, true, ANGLE_MAX, ANGLE_MIN, control_sampling_time);

// Tset = 6.5s ~ 9s
float Kc = 0.052844;
float Td = 2.4056;

float distance_INPUT;
float servo_angle_OUTPUT;

float distance_REF = 177.0;
float distance_ERROR = 0.0;

float distance_sampling_time = 15;
float control_sampling_time = 60;

float last_distance_sample, last_control_sample;

// FSM =============================================================
typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

enum{
  sm_init = 0,
  sm_manual,
  sm_direct,
  sm_done
};

fsm_t test_fsm;

bool I, P, M, D, B, X, W, Y, Z;

// ==================================================================


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

float takeDistanceAverage(){
  float dist_sum = 0.0;
  float real_dist = 0.0;
  for (int i = 0; i < 4; i++){
    dist_sum = dist_sum + distance[i];
  }

  dist_sum = dist_sum/4.0;
 
  if (dist_sum < 2.0) {
    real_dist = 0.0;
  } else if (dist_sum <250.0){
    real_dist = dist_sum - 5.0;
  } else if (dist_sum < 500) {
    real_dist = dist_sum*0.3278 + 149.1;
  } else {
    real_dist = 400.0;
  }

  return real_dist;

}

float rad2command(float rad){
  float deg = rad*180.0/PI;
  return (508.7613 - 41.0499*deg - 0.1267*(deg*deg));
}

#if MAIN

void setup() {
  
  Serial.begin(115200);
  controller.setProportional(Kc);
  controller.setDerivative(Td);
  if (!lox.begin()) {
    Serial.println("Failed to find VL53L0X sensor!");
    while (1) {
      // Stay here if sensor not detected
    }
  }
  distance_sampling_time = 50;
  control_sampling_time = 1000;

}

void loop() {
  
  now = millis();
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print(".");
  //delay(10);
  //SAMPLE DISTANCE
  if (now - last_distance_sample > distance_sampling_time){
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance[dist_index] = float(measure.RangeMilliMeter);
      //Serial.println("Distance (mm): " + String(distance[dist_index])); 
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
    //Serial.println("Integral sum = " + String(controller.integral.sum) + ", Last derivative = " + String(controller.derivative.last_input));
    //delay(50);
    //Serial.print("-> ");
    //Serial.println("Step size = " + String(controller.integral.step_size) + "; Kc = " + String(controller.Kc) + "; Td = " + String(controller.Td));
    //delay(50);
    Serial.println(" ");
    Serial.print("Time = " + String(now));

    distance_INPUT = takeDistanceAverage();
    distance_ERROR = distance_REF - distance_INPUT;
    servo_angle_OUTPUT = rad2command(controller.computeOutput(distance_ERROR));    

    Serial.println("Input = " + String(distance_INPUT) + " | Error = " + String(distance_ERROR) + " | Output = " + String(servo_angle_OUTPUT));

    //write servo_angle_OUTPUT on the servo motor (...)
    servoCont.moveWithTime(1, servo_angle_OUTPUT, 60);

    last_control_sample = now;
  }
}

#endif

#if TEST_ANGLE
void setup() {
  //lox.begin();
  Serial.begin(115200);
  Wire.begin();
  distance_sampling_time = 100;
  control_sampling_time = 500;

  if (!lox.begin()) {
    Serial.println("Failed to find VL53L0X sensor!");
    while (1) {
      // Stay here if sensor not detected
    }
  }
}

void loop() {
    
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  now = millis();

  //SAMPLE DISTANCE
  if (now - last_distance_sample > distance_sampling_time){
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance[dist_index] = float(measure.RangeMilliMeter);
      //Serial.println("Distance (mm): " + String(distance[dist_index])); 
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

  if (now - last_control_sample > control_sampling_time){
      
      distance_INPUT = takeDistanceAverage();
      //Serial.println("Distance average (mm): " + String(distance_INPUT));
      Serial.println("=====");
      last_control_sample = now;
  }


}

#endif

#if TEST_COMMANDS
void setup() {
  Wire.begin();
  delay(1000);
  lox.begin();
  serial_motors.begin(115200);
  Serial.begin(115200);
  mpu.initialize();
  
  for (int i = 0; i<96; i++) {
    measured_commands_angle[i] = 0.0;
  }
  test_index = 0;
  command = 250.0;

  avg_angle = 0.0;
  pitch = 0.0;

  delay(10000);

  Serial.println("Starting test...");

}

void loop() {


  now = millis();
  if ((now - last_test_time > test_interval) && (test_index < 96)){

    //Write servo angle on the servo motor and wait for its stabilization
    servoCont.moveWithTime(1, command, 60);
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
    measured_commands_angle[test_index] = pitch;
    Serial.println("[i = " + String(test_index) + "]: Servo Command = " + String(command) + "; Measured Angle = " + String(pitch));
    
    test_index++;

    command += command_step;

    last_test_time = now;
  } else Serial.println("now = " + String(now));

  if ((test_index == 96) && !done){
    done = true;
    command = 250.0;
    for (int i = 0; i<96;i++){
      Serial.println(String(command) + ", " + String(measured_commands_angle[i]));
      command += command_step;
    }
  }

}

#endif

#if MANUAL
void setup() {
  lox.begin();
  serial_motors.begin(115200);
  Serial.begin(115200);
  mpu.initialize();
  Wire.begin();
  avg_angle = 90.0;
  set_state(test_fsm, sm_init);
  test_interval = 100;
}

void loop(){


  if (Serial.available() > 0){
    uint8_t c = Serial.read();
    if (c == '+') P = true;
    else if (c == '-') M = true;
    else if (c == 'I') I = true;
    else if (c == 'D') D = true;
    else if (c == 'B') B = true;
    else if (c == 'X') X = true;
    else if (c == 'Y') Y = true;
    else if (c == 'Z') Z = true;
    else if (c == 'W') W = true;
  }

  now = millis();

  if (now - last_test_time > test_interval){

    if (test_fsm.state == sm_init && M){
      manual_angle = 90.0;
      test_fsm.new_state = sm_manual;
    } else if (test_fsm.state == sm_init && D){
      test_fsm.new_state = sm_direct;
      test_index = 0;
      command = 250.0;
    } else if (test_fsm.state == sm_manual && D){
      test_fsm.new_state = sm_direct;
      test_index = 0;
      command = 250.0;
    } else if (test_fsm.state == sm_manual && B){
      test_fsm.new_state = sm_init;
    } else if (test_fsm.state == sm_direct && B){
      test_fsm.new_state = sm_init;
    } else if (test_fsm.state == sm_direct && test_index == 96){
      test_fsm.new_state = sm_done;
    } else if (test_fsm.state == sm_done){
      test_fsm.new_state = sm_init;
      test_index = 0;
      command = 250.0;
    }
  
    set_state(test_fsm, test_fsm.new_state);
  
    switch (test_fsm.state)
    {
    case sm_init:
      avg_angle = 0.0;
      for (int i = 0; i<5; i++){
        readIMU();
        avg_angle += pitch;
        if (i!=4) delay(20);
      }
      pitch = avg_angle/5.0;
      Serial.println("Pitch: " + String(pitch));

      //MAX = 260 = 10º
      //MIN = 730 = -10

      break;
    case sm_manual:
      if (P) manual_angle += 5.0;
      if (M) manual_angle -= 5.0;
      if (X) manual_angle = 250;
      if (W) manual_angle = 400;
      if (Y) manual_angle = 600;
      if (Z) manual_angle = 700;

      servoCont.moveWithTime(1, manual_angle, 60);
      delay(60);
      avg_angle = 0.0;
      for (int i = 0; i<5; i++){
        readIMU();
        avg_angle += pitch;
        if (i!=4) delay(20);
      }
      pitch = avg_angle/5.0;
      Serial.println("Pitch (MEASURED): " + String(pitch));
      Serial.println("Pitch (MANUAL) = " + String(manual_angle));
      

    //

      break;
    case sm_direct: 
      servoCont.moveWithTime(1, command, 60);
      delay(100);
      //Measure pitch angle (take average)
      avg_angle = 0.0;
      for (int i = 0; i<5; i++){
        readIMU();
        avg_angle += pitch;
        if (i!=4) delay(30);
      }

      pitch = avg_angle/5.0;
      
      //Store measured angle
      measured_commands_angle[test_index] = pitch;
      test_index++;
      command += command_step;
      delay(50);
      Serial.println("[i = " + String(test_index) + "]: Servo Command = " + String(command) + "; Measured Angle = " + String(pitch));
      

      
      break;
    case sm_done:
      command = 250.0;


      break;
    default:
      break;
    }
  
    P = M = I = D = B = X = Y = W = Z = false;

    last_test_time = now;
  }

}


#endif

