#include <Arduino.h>

// required for I2C
#include <Wire.h>

// put function declarations here:
int myFunction(int, int);
void checkMagnetPresence();


//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing
float encoderTimer = 0;


void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(115200);
  Wire.begin(); //start i2C  
	Wire.setClock(400000L); 


}

void loop() {
  // put your main code here, to run repeatedly:
}

void checkMagnetPresence()
{
  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
    {
      magnetStatus = 0; //reset reading

      Wire.beginTransmission(0x36); //connect to the sensor
      Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
      Wire.endTransmission(); //end transmission
      Wire.requestFrom(0x36, 1); //request from the sensor

      while(Wire.available() == 0); //wait until it becomes available 
      magnetStatus = Wire.read(); //Reading the data after the request

      Serial.print("Magnet status: ");
      Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
    }      
    
    //Status register output: 0 0 MD ML MH 0 0 0  
    //MH: Too strong magnet - 100111 - DEC: 39 
    //ML: Too weak magnet - 10111 - DEC: 23     
    //MD: OK magnet - 110111 - DEC: 55

    Serial.println("Magnet found!");
    delay(500);  
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}