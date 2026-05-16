//part-2
#include <Braccio.h>
#include <Servo.h>


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;


void setup() {  
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:
  //Base (M1):90 degrees
  //Shoulder (M2): 45 degrees
  //Elbow (M3): 180 degrees
  //Wrist vertical (M4): 180 degrees
  //Wrist rotation (M5): 90 degrees
  //gripper (M6): 10 degrees
  Braccio.begin();
}

void loop() {
  delay(12000);

  Braccio.ServoMovement(20,           0,  30, 180, 90,  90,  10);
  
  //Wait 1 second
  delay(1000);

  Braccio.ServoMovement(20,           0,  30, 180, 90,  90,  120);

  delay(1000);

  Braccio.ServoMovement(20,           60,  30, 180, 90,  90,  120);

  delay(1000);

  Braccio.ServoMovement(20,           60,  30, 180, 10,  90,  120);

  delay(1000);

  Braccio.ServoMovement(20,           60,  30, 180, 10,  90,  10);
}