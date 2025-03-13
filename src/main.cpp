#include <Arduino.h>
#include "controller.h"
#include <Adafruit_VL53L0X.h>
#include <WString.h>

#define ANGLE_MAX 30
#define ANGLE_MIN -30

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
float distance[] = {0.0, 0.0, 0.0, 0.0};

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

float distance_REF = 10.0;
float distance_ERROR = 0.0;


Controller controller("servo angle controller", true, false, true, ANGLE_MAX, ANGLE_MIN, control_sampling_time);

void setup() {
  
  lox.begin();

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
    servo_angle_OUTPUT = controller.computeOutput(distance_ERROR);    

    //write servo_angle_OUTPUT on the servo motor (...)

    last_control_sample = now;
  }
  

}


float takeDistanceAverage(){
  float dist_sum = 0.0;

  for (int i = 0; i < 4; i++){
    dist_sum = dist_sum + distance[i];
  }

  return dist_sum/4.0;
}

