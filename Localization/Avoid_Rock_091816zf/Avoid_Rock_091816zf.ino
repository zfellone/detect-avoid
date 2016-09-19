/*
http://pulsedlight3d.com
LIDAR-Lite Continuous Pulse-Width Modulation
The LIDAR-Lite has the ability to read distance without the use of I2C, simply 
by reading the amount of time a signal from the mode pin is HIGH. We then take that
duration (here measured in microseconds) and convert it to centimeters using the 
LIDAR-Lite constant of 10usec/cm. We then print that value to the serial monitor. 
Technical explanation is on page 15/16 of the operating manual: 
http://pulsedlight3d.com/pl3d/wp-content/uploads/2014/11/LIDAR-Lite-Operating-Manual.pdf
*/

//gittest

#include <Servo.h>
#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define ir A0
#define model 20150

long pulse_width;
long zero = 30;
long difference;
long obDistance;
bool sweeped = false;
long correction = 13;
int dis;
int buf;

Servo l;
Servo r;
Servo sweep;

SharpIR sharp(ir, 25, 93, model);

int sweepPos = 0;

int rSharp = A0;
int rSharpValue = 0;

int count = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{

  Serial.begin(9600); // Start serial communications

  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  pinMode(ir, INPUT);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
    
  bno.setExtCrystalUse(true);

  l.attach(6);
  r.attach(9);
  sweep.attach(10);
  l.write(90);
  r.write(90);
  sweep.write(90);
  delay(500);
}

void loop()
{
  r.write(60);
  l.write(120);
  if(detectedRock()){
    avoid();
  }
  else if(detectedCrater()){
    while(1==1){
      l.write(90);
      r.write(90);
    }
  }
}

void avoid(void){
  sensors_event_t event;
  bno.getEvent(&event);
  int angle_start = event.orientation.x;
  int current_angle = angle_start;
  int angle_target = angle_start - 90;
  if(angle_target < 0){
    angle_target += 360;
  }
  Serial.print("Start angle:  ");
  Serial.println(angle_start);
  
  while(current_angle < (angle_target - 2) || current_angle > (angle_target + 2)){
    l.write(40);
    r.write(40);
    Serial.println("Turning");
    bno.getEvent(&event);
    current_angle = event.orientation.x;
    Serial.println(current_angle);
  }
  
  dis=sharp.distance();
  buf = dis+10;
  while(dis<buf){
    l.write(60);
    r.write(120);
    dis = sharp.distance();
  }
  delay(100);

  angle_target = current_angle + 90;
  if(angle_target > 360){
    angle_target -= 360;
  }
  Serial.print("Start angle:  ");
  Serial.println(current_angle);

  while(current_angle < (angle_target - 2) || current_angle > (angle_target + 2)){
    l.write(140);
    r.write(140);
    Serial.println("Turning");
    bno.getEvent(&event);
    current_angle = event.orientation.x;
    Serial.println(current_angle);
  }
}

  /*do{
    l.write(60);
    r.write(120);
    Serial.println("Backing up");
  }while(detected());
  sweep.write(90);

  while(1==1){
    l.write(90);
    r.write(90);
  }*/

bool detectedRock(void){
  pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Liteinch
        pulse_width = pulse_width - correction;
        difference = pulse_width - zero;
  }
  if(difference < -20){
            Serial.print("Obstacale!!");
            Serial.print("   ");
            Serial.println(difference);
            return true; 
    }
  
  else{ 
    return false;
  }
 }

 bool detectedCrater(void){
  pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Liteinch
        pulse_width = pulse_width - correction;
        difference = pulse_width - zero;
  }
  if(difference > 10){
            Serial.print("Obstacale!!");
            Serial.print("   ");
            Serial.println(difference);
            return true; 
    }
  
  else{ 
    return false;
  }
 }

