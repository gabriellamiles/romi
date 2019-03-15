#ifndef _IRSensor_h
#define _IRSensor_h

class IRSensor 
{
  public:
    //Public variables and methods go here

    // define IRSensor
    IRSensor(int pin);

    // read the raw data value of the IR Sensor, should be between 0 - 1023
    int raw_infrared();
    
    // convert the raw data to a distance (in mm) measurement from the obstacle in front of the sensor
    float update(int raw_data);

  private:
    //Private variables and methods go here
    
    int pin;
    int raw_IR;
    float dist_mm;
  
};

IRSensor::IRSensor(int pin) {
  // define IRSensor
  pinMode(pin, INPUT);
}

int IRSensor::raw_infrared() {
  // read the raw data value of the IR Sensor, should be between 0 - 1023
  int raw_IR = analogRead(pin);
  return raw_IR;
  
}

float IRSensor::update(int raw_IR) {
  // convert the raw data to a distance (in mm) measurement from the obstacle in front of the sensor
  //float dist_mm = pow(((raw_IR*0.00489236791)/21.941), (1/(-0.793)))*10;

  float dist_pt1 = ((raw_IR*0.00489236791)/21.941);
  float dist_pt2 = (1/(-0.793));
  float dist_mm = pow(dist_pt1, dist_pt2)*10;

  return dist_mm;
}

#endif
