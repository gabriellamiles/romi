#include "ir_sensor.h"
#include "pid.h"
#include "encoders.h"

#define IR_SENSOR A1
#define LOOP_DELAY 50
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

//declare global variables
byte l_speed;
byte r_speed;
int target_dist = 130; //mm (from any obstacle)


// *** Classes *** //

IRSensor irSensor(IR_SENSOR);

/* PID controller information see pid.h*/
float Kp_pose_left = 0.05; //Proportional gain for position controller
float Kd_pose_left = 0.06; //Derivative gain for position controller
float Ki_pose_left = 0.0; //Integral gain for position controller
PID leftPose(Kp_pose_left, Kd_pose_left, Ki_pose_left); // controller for left wheel position, count_e1
float Kp_pose_right = 0.085; //Proportional gain for position controller
float Kd_pose_right = 0.06; //Derivative gain for position controller
float Ki_pose_right = 0.0; //Integral gain for position controller
PID rightPose(Kp_pose_right, Kd_pose_right, Ki_pose_right); // controller for right wheel position, count_e0


void setup() {
  // put your setup code here, to run once:

  //assign motor pins and set direction
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  //set initial speed of wheels
  l_speed = 0;
  r_speed = 0;

  //set up pin change interrupts for the encoders
  setupEncoder0();
  setupEncoder1();

  //intialise Serial communication
  Serial.begin(9600);
  Serial.println("******RESET*****"); //for debugging purposes

  pinMode(IR_SENSOR, INPUT);

}

void loop() {

  //this works but there is a slight discrepancy between the results from the class 
  // and the results from the function for some reason - ask Paul/Martin?
  
//  int raw_IR = irSensor.raw_infrared();
//  float dist_from_obs = irSensor.update(raw_IR);
//  Serial.print("Distance class: ");
//  Serial.println(dist_from_obs);

  float dist_from_obs = calculate_IR_distance();
  Serial.print("Distance: ");
  Serial.println(dist_from_obs);

  /*obstacle_avoidance(dist_from_obs);*/

  float output_left = leftPose.update(target_dist, dist_from_obs);
  Serial.print("Left wheel output: ");
  Serial.println(output_left);

  float output_right = rightPose.update(target_dist,dist_from_obs);
  Serial.print("Right wheel output: ");
  Serial.println(output_right);

  if (output_left > 0) {
    drive_backwards(output_left,output_right);
  } else if (output_left < 0) {
    drive_forwards(output_left,output_right);
  } else {
    stay_still();
  }
  
  delay(LOOP_DELAY);
  
  
}

/////////////////////// INFRARED SENSOR FUNCTIONS //////////////////

float calculate_IR_distance() {
  
  int raw_IR_manual = analogRead(IR_SENSOR);
  float dist_pt1 = ((raw_IR_manual*0.00489236791)/21.941);
  float dist_pt2 = (1/(-0.793));
  float dist_mm = pow(dist_pt1, dist_pt2)*10;

  return dist_mm;
  //float dist_mm = pow(((raw_IR*0.00489236791)/21.941), (1/(-0.793)))*10;

  
}

void obstacle_avoidance(float obj_dist) {
  if (obj_dist < 100){
    drive_backwards_a();
  } else if (obj_dist > 120) {
    drive_forwards_a();
  } else {
    stay_still();
  }

}


//////////////////// DRIVING FUNCTIONS /////////////////////////

void drive_backwards_a() {
  digitalWrite(L_DIR_PIN, HIGH);
  digitalWrite(R_DIR_PIN, HIGH);
  l_speed = 20;
  r_speed = 20;
  analogWrite(L_PWM_PIN, l_speed);
  analogWrite(R_PWM_PIN, r_speed);
}

void stay_still(){
  digitalWrite(L_DIR_PIN, HIGH);
  digitalWrite(R_DIR_PIN, HIGH);
  l_speed = 0;
  r_speed = 0;
  analogWrite(L_PWM_PIN, l_speed);
  analogWrite(R_PWM_PIN, r_speed);
}

void drive_forwards_a(){
  digitalWrite(L_DIR_PIN, LOW);
  digitalWrite(R_DIR_PIN, LOW);
  l_speed = 20;
  r_speed = 20;
  analogWrite(L_PWM_PIN, l_speed);
  analogWrite(R_PWM_PIN, r_speed);
}

/////// DRIVING WITH PID INPUT ////////

void drive_forwards(float output_left, float output_right){
  
  digitalWrite(L_DIR_PIN, LOW);
  digitalWrite(R_DIR_PIN, LOW);
  l_speed = abs(output_left);
  r_speed = abs(output_right);
  analogWrite(L_PWM_PIN, l_speed);
  analogWrite(R_PWM_PIN, r_speed);
  
}

void drive_backwards(float output_left, float output_right) {
  digitalWrite(L_DIR_PIN, HIGH);
  digitalWrite(R_DIR_PIN, HIGH);
  l_speed = abs(output_left);
  r_speed = abs(output_right);
  analogWrite(L_PWM_PIN, l_speed);
  analogWrite(R_PWM_PIN, r_speed);
}
