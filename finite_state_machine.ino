#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"

#define LOOP_DELAY 100
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BUZZER_PIN 6
#define BAUD_RATE = 115200;
#define wheel_base 165

#define LINE_LEFT_PIN A3
#define LINE_CENTRE_PIN A4
#define LINE_RIGHT_PIN A2

#define STATE_INITIAL 0
#define STATE_DRIVE_FORWARDS 1
#define STATE_FOUND_LINE 2
#define STATE_FOLLOW_LINE 3
#define STATE_END_LINE 4
#define STATE_CORRECT_HEADING 5

////////////////////////// GLOBAL VARIABLES //////////////////////

int STATE;
int number_of_buzzes;
int number_of_line_buzzes;

float Kp_pose_left = 0.05; //Proportional gain for position controller
float Kd_pose_left = 0.06; //Derivative gain for position controller
float Ki_pose_left = 0.0; //Integral gain for position controller
PID leftPose(Kp_pose_left, Kd_pose_left, Ki_pose_left); // controller for left wheel position, count_e1

float Kp_pose_right = 0.085; //Proportional gain for position controller
float Kd_pose_right = 0.06; //Derivative gain for position controller
float Ki_pose_right = 0.0; //Integral gain for position controller
PID rightPose(Kp_pose_right, Kd_pose_right, Ki_pose_right); // controller for right wheel position, count_e0

Line_Sensor lineLeft(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
Line_Sensor lineCentre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
Line_Sensor lineRight(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics kinematics_l;
Kinematics kinematics_r;


///////////////////////////////////////////////////////////////////

void setup() {
  //Assign motor pins and set direction
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

  //initialise the serial communication
  
  Serial.begin( 9600 );
  Serial.println("***RESET***");

  //set initial state, before robot operation
  STATE = STATE_INITIAL;

}

void loop() {
  
  // put your main code here, to run repeatedly:
  switch(STATE) {
    
    case STATE_INITIAL: // state 0
      initialisingBeeps();
      break;
      
    case STATE_DRIVE_FORWARDS: // state 1
      driveForwards();
      break;
      
    case STATE_FOUND_LINE: // state 2
      drive_distance(0,0,200,200);
      foundLineBeeps();
      break;
      
    case STATE_FOLLOW_LINE:  // state 3
      lineFollow();
      break;

    case STATE_END_LINE: // state 4
      drive_distance(0,0,200,200);
      longBeep();
      Serial.println("Case 4!");
      //wait for 5 seconds

    case STATE_CORRECT_HEADING: // state 5
      drive_distance(20,20,0,0);
      Serial.println("Case 5!");
      
    default:
      Serial.println("System error, unknown state!");
      break;
  }

  //small delay
  delay(10);

}


/////////////////////////////////////////////// STATE 0 - all perfect, do not touch ///////////////////////////

void initialisingBeeps() {
  if (number_of_buzzes < 5) {
    play_tone(50,100);
    number_of_buzzes++;
    Serial.println(number_of_buzzes);
    delay(1000);
  } else {
    STATE++;
  }
}

void play_tone(int volume, int duration) {
  analogWrite(BUZZER_PIN, volume); // turn on buzzer
  delay(duration);
  analogWrite(BUZZER_PIN, 0); //turn buzzer off
}

////////////////////////////////////////////// STATE 1 - completed //////////////////////

void driveForwards() {
  
  float target = -5000;
  
  float output_left = leftPose.update(target, count_e1); //
  /*Serial.print("Left wheel output is: ");
  Serial.println(output_left);*/
  delay(LOOP_DELAY);
  float output_right = rightPose.update(target, count_e0);
  /*Serial.print("Right wheel output is: ");
  Serial.println(output_right);*/
  delay(LOOP_DELAY);
  drive_distance(output_left*0.5, output_right*0.5, target, target);

  int left = leftSensorCheck();
  int right = rightSensorCheck();
  int centre = centreSensorCheck();

  if (left == 1 || right == 1 ||centre == 1) {
    Serial.println("found line!");
    STATE++;
  }

  Serial.println("left: ");
  Serial.println(left);
}

///////////////////////////////////////////////// STATE 2 - done! /////////////////////////////

void foundLineBeeps(){
  if (number_of_line_buzzes < 2) {
    play_tone(50,100);
    number_of_line_buzzes++;
    Serial.println(number_of_line_buzzes);
    delay(1000);
  } else {
    STATE++;
  }
}

//////////////////////////////////////////////// STATE 3 - in progress ////////////////////////


void lineFollow(){
  
  int left = leftSensorCheck();
  int right = rightSensorCheck();
  int centre = centreSensorCheck();

  BangBang(left, right, centre);  
  
}

void BangBang (int left, int right, int centre) {

  float x_pos = kine_track_x();
  Serial.println("X_pos: ");
  Serial.println(x_pos);
  
  if ( left == 1 && right == 1 && centre == 1) {
     //Serial.println("line detected");
     drive_distance(20,20,x_pos-1,x_pos-1);
     
  } else if (left == 1 && right == 0) {
    //stop
    drive_distance(0,0, x_pos, x_pos);
    rotate_left();
    //turn left
    
  } else if (right == 1 && left == 0 ){
    //stop
    drive_distance(0,0,x_pos,x_pos);
    //turn right
    rotate_right();
    
  } else if (right == 0 && left == 0 && centre == 0) {
    STATE++;
  }
   
}

int kine_track_x () {
  
  // calculate how much each wheel has moved since the last update (in mm?)
  float heading_left = kinematics_l.update(count_e1);
  float heading_right = kinematics_r.update(count_e0);
  
  //calculate the average distance moved by the centre of the board
  float d_calc = kinematics_l.dist_calc(heading_right,heading_left);
  
  //calculate the angle that the romi moves through since last update
  float angle_new = kinematics_l.angle_moved(heading_left, heading_right, wheel_base);
  
  //record distance moved (in x-direction)
  float total_dist_x = kinematics_l.dist_counter_x(d_calc,angle_new);

  return total_dist_x;
  
}

void rotate_left(){
    digitalWrite(L_DIR_PIN, HIGH);
    digitalWrite(R_DIR_PIN, LOW);
    l_speed = abs(20);    
    r_speed = abs(20);
    analogWrite(L_PWM_PIN, l_speed);
    analogWrite(R_PWM_PIN, r_speed);  
}

void rotate_right(){
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_DIR_PIN, HIGH);
    l_speed = abs(20);    
    r_speed = abs(20);
    analogWrite(L_PWM_PIN, l_speed);
    analogWrite(R_PWM_PIN, r_speed);  
}

void flash_yellow_led (){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);  

}

//////////////////////////////// STATE 4 - end line, pause, make noise ///////////////

void longBeep() {
  if (number_of_buzzes < 8) {
    play_tone(50,100);
    number_of_buzzes++;
    delay(1000);
  } else {
    STATE++;
  }
}

/////////////////////////////// MULTI_USE FUNCTIONS /////////////////////////////////

int leftSensorCheck(){
  int output_lineLeft = lineLeft.read_raw();
  int calib_lineLeft = lineLeft.calibrate(output_lineLeft);
  int line_find_left = lineLeft.line_detection(calib_lineLeft); 
  return line_find_left;
}
int rightSensorCheck(){
  int output_lineRight = lineRight.read_raw();
  int calib_lineRight = lineRight.calibrate(output_lineRight);
  int line_find_right = lineRight.line_detection(calib_lineRight);
  return line_find_right;
  
}
int centreSensorCheck(){
  int output_lineCentre = lineCentre.read_raw();
  int calib_lineCentre = lineCentre.calibrate(output_lineCentre);
  int line_find_centre = lineCentre.line_detection(calib_lineCentre);
  return line_find_centre;
}
