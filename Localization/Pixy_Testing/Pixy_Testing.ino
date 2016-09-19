#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Pixy pixy;

double P1Dist, P2Dist;
double P1Theta, P2Theta;
int Calibrated;
double Origin = 0.0;

/**************************************************************************/
 /*  
    Determines the current system calibration status and returns it as an 
    integer 0-3 with 0 being not calibrated and 3 being fully calibrated
*/
/**************************************************************************/

int Cal_Status(void){   
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return (int)system;
}

/**************************************************************************/
 /*  
    Determines the distance reading from the LIDAR and converts it
    Returns a double precision floating point distance in inches
*/
/**************************************************************************/
double Distance(void){
  int reading = 0;
  double newread = 0;
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  newread = (((double)reading)/2.54);
  return newread;
}
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
 /*  
    Sets up the initial parameters, only runs once
*/
/**************************************************************************/
void setup() {
Serial.begin(9600);   //Begins a serial port running at 9600 baud
pixy.init();          //Initialises the pixy camera

  /* Initialise the IMU */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

Serial.println("Initialized");

  bno.setExtCrystalUse(true);

}

void loop() {

while(Calibrated==0){               //While the IMU isn't calibrated, wait for it to be calibrated (This is where we would need an automated routine)
    Calibrated = Cal_Status();
    Serial.println("Calibrating");
    delay(100);
  }

  if (pixy.getBlocks())             //If the pixy can see a color that it recognises, then:
  {
    if(pixy.blocks[0].signature == 1); //If color signature 1 is being seen, then:
    {
      if(pixy.blocks[0].x == 160)   //If color signature 1 is in the center of the FOV then:
      {
        sensors_event_t event;    //Initialize event
        bno.getEvent(&event);     //put event data into the sensor event stucture
        P1Theta = event.orientation.x;  //read the x orientation of the IMU from the event structure and store it as the Theta for the first signature
        P1Dist = Distance();      //Get the distance from the LIDAR and store it as the distance of the first structure
        delay(BNO055_SAMPLERATE_DELAY_MS);  // Wait the required refresh time so we don't over request the sensor
        Serial.print("Distance 1: "); //Display the distance to the console
        Serial.print(P1Dist);
        Serial.print("\t Theta 1: "); // Display the relative angle to the console
        Serial.println(P1Theta);
      }
    }
    if(pixy.blocks[1].signature == 1); //Do the same for the second signature
      {
       if(pixy.blocks[1].x == 160)
      {
        sensors_event_t event;
        bno.getEvent(&event);
        P2Dist = Distance();
        P2Theta = event.orientation.x;
        delay(BNO055_SAMPLERATE_DELAY_MS);
        Serial.print("Distance 2: ");
        Serial.print(P2Dist);
        Serial.print("\t Theta 2: ");
        Serial.println(P2Theta);
      }
    }
  }
  
  //if we have data for both points then calculate the angle at which the sensor is parallel to the wall
  if ((P1Dist != 0) &&(P1Theta != 0)&&(P2Dist != 0) && (P2Theta != 0) && (Origin == 0)) {
    double Theta, ATheta, A, B, dist, x, y;

    Theta = (PI/180)*abs(P1Theta-P2Theta);
    Serial.print("Theta");
    Serial.println(Theta);
    
    if (P1Dist > P2Dist) {
      A = P2Dist;
      B = P1Dist;
      ATheta = P2Theta;
    }
    else{
      A = P1Dist;
      B = P2Dist;
      ATheta = P1Theta;
    }
    
    Origin = acos((B-(A*cos(Theta)))/sqrt((square(B)+square(A)-(2*B*A*cos(Theta)))));
    Origin = 90-(180/PI*(Origin+Theta));
    Origin = Origin + ATheta;
    Serial.print("Original Angle: "); //Print the angle that the sensor is parallel to the wall to the console
    Serial.println(Origin);

    while (1==1){ //After we have calibrated, do this indefinitely:
       sensors_event_t event;
        bno.getEvent(&event);
        dist = Distance();
        Theta = event.orientation.x-Origin; //Determine the angle relative to the origin angle
        Serial.print("Angle: ");
        Serial.print(Theta);
        Serial.print("\t Distance: ");
        Serial.print(dist);
        x = dist*sin(Theta*(PI/180));   //Convert from cylindrical coordinates to cartesian
        y = dist*cos(Theta*(PI/180));
        Serial.print("\t X: ");
        Serial.print(x);
        Serial.print("\t Y: ");
        Serial.println(y);
        delay(100);
    }
  }
}
