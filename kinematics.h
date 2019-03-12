#ifndef _Kinematics
#define _Kinematics_h

class Kinematics
{
  public:
    //Public variables and methods go here
    
    void Kinematics::reset();
    
    float Kinematics::update(float measurement);

    float Kinematics::dist_calc(float heading_l, float heading_r);
    
    float Kinematics::angle_moved(float heading_left, float heading_right, int wheel_base);
    
    float Kinematics::dist_counter_x(float distance_updated, float angle);

    float Kinematics::dist_counter_y(float distance_updated, float angle);
    
  private:
    //Values to store

    float last_measurement=0; //For storing the last measurement
    float angle_old = 0;

    
    float last_distance_x = 0; //(always starting from 0 in WCS)
    float total_x = 0;
    float last_distance_y = 0;
    float total_y = 0;

    float last_error = 0;
    long last_millis = 0;
    
    bool task_complete = false; // controls whether current task is completed or not
    bool debug=false; //This flag controls whether we print the contributions of each component when update is called
    bool show_response = false; // This flag controls whether we print the response of the controller on each update
   
};

float Kinematics::update(float measurement) {
  
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  int time_delta = time_now - last_millis;
  last_millis = time_now;

  float current_measurement = measurement;

  float measurement_delta = ((current_measurement - last_measurement))*0.15; 
  //previous error for calculating error derivative

  /*Serial.println("Measurement: ");
  Serial.println(measurement);
  Serial.println(current_measurement);
  Serial.println(last_measurement);*/
  
  last_measurement = measurement;

  return measurement_delta;

}

float Kinematics::dist_calc(float heading_l, float heading_r){

  float d = (heading_r+heading_l)/2;
  return d;
  
      
}

float Kinematics::angle_moved(float heading_left, float heading_right, int wheel_base){
  
  float angle_new = angle_old + (heading_left-heading_right)/wheel_base;
  angle_old = angle_new;
  return angle_new;  
  
}

float Kinematics::dist_counter_x(float distance_updated, float angle){

  //calculate new total
  total_x = last_distance_x + distance_updated*cos(angle);
  
  //store old value
  last_distance_x = total_x;
  
  return total_x;
  
}

float Kinematics::dist_counter_y(float distance_updated, float angle){

  //calculate new total
  total_y = last_distance_y + distance_updated*sin(angle);
  
  //store old value
  last_distance_y = total_y;
  
  return total_y;
  
}


#endif
