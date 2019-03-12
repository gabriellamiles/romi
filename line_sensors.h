#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
 
class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int pin);
    //Calibrate
    int calibrate(int uncal_read);
    //Return the uncalibrated value from the sensor
    int read_raw();
    //Return the calibrated value from the sensor
    void read_calibrated();
    //determine whether or not there is a line
    int line_detection(int reading);
    
  private:
  
    int pin;
    int last_read = 0;
    int stored_read = 0;
    int fin_storage = 0;
    int threshold = 600;
    int line_dect;
    /*
     * Add any variables needed for calibration here
     */
    
};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

int Line_Sensor::read_raw()
{
  return analogRead(pin);
}

int Line_Sensor::calibrate(int uncal_read)
{
  
  fin_storage = stored_read + uncal_read;
  stored_read = uncal_read + last_read;
  last_read = uncal_read;

  return (fin_storage/3);
}

void Line_Sensor::read_calibrated()
{
  
  /*
   * Write code to return a calibrated reading here
   */
}

int Line_Sensor::line_detection(int reading){

  if (reading > threshold) {
    line_dect = 1;
    return line_dect;
  } else if (reading <= threshold){
    line_dect = 0;
    return line_dect;
  }
   
}


#endif
