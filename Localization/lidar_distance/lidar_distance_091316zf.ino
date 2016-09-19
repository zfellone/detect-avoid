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

#include <Servo.h>

unsigned long pulse_width;
unsigned long zero = 40;
long difference;
long obDistance;
bool deteced;
bool sweeped = false;

Servo l;
Servo r;
Servo sweep;

int sweepPos = 0;

int rSharp = A0;
int rSharpValue = 0;

int count = 0;

void setup()
{
  Serial.begin(9600); // Start serial communications
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read

  l.attach(6);
  r.attach(9);
  sweep.attach(10);
  sweep.write(90);
}

void loop()
{
  pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Liteinch
        difference = pulse_width - zero;
        if(difference < -10 || difference > 10){
            Serial.print("Obstacale!!");
            Serial.print("   ");
            Serial.println(difference);
            obDistance = difference;
            deteced = true; 
        }
        else{
            deteced = false;
            sweeped = false;
        }
        if(deteced == false){
            l.write(120);
            r.write(60);
        }
        else{
            l.write(120);
            r.write(120);
            }
        }
  
  delay(20); //Delay so we don't overload the serial port
}
