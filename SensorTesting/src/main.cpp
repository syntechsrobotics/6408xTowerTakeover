/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RangeFinderE         sonar         E, F            
// RangeFinderG         sonar         G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "vision.h"
using namespace vex;

// A global instance of competition
competition Competition;
vex::motor FrontLeft_Motor = vex::motor(vex::PORT13, false);
vex::motor FrontRight_Motor = vex::motor(vex::PORT17, true);
vex::motor BackLeft_Motor = vex::motor(vex::PORT6, false);
vex::motor BackRight_Motor = vex::motor(vex::PORT18, true);
vex::controller Controller = vex::controller();
vex::motor Ramp_Motor = vex::motor(vex::PORT14, true);
vex::motor Arm_Motor = vex::motor(vex::PORT19, true);
vex::motor Left_Intake_Motor = vex::motor(vex::PORT12, true);
vex::motor Right_Intake_Motor = vex::motor(vex::PORT20, false);
vex::vision Vision = vex::vision(vex::PORT15);

const double TURNING_RATE = 1.0;
const double MOVING_RATE = 1.0;
const double MAX_SPEED = 100.0;

const int HZ_15_WAIT = 1000 / 15;
const int VISION_WIDTH = 312;
const int VISION_HEIGHT = 210;
const int VISION_MAX_DISTANCE = 30000;
const int VISION_OFFSET = 0;

bool button_switch = true;

const int PURPLE = 0;
const int GREEN = 1;
int level_count = 0;
int DR4B_LEVEL = 0;

float speed_L = 0;
float speed_R = 0;
//void LimitSwitchPressed() { Arm_Motor.stop(); }

// define your global instances of motors and other devices here
// Absolute value function, i.e: x=|x|
float abs(float num) {
  if (num > 0) {
    return num;
  } else {
    return -num;
  }
}

// forces an abs() number between abs() the min and max values. Note: Keeps the
// sign of the number
double capMinMax(double val, double min, double max) {
  if (val < min && val > 0) {
    val = min;
  } else if (val > -min && val < 0) {
    val = -min;
  }
  if (val > max) {
    val = max;
  } else if (val < -max) {
    val = -max;
  }
  return val;
}
int Ramp_power() {
  float Ramp_Speed = 100 - Ramp_Motor.rotation(rotationUnits::deg)/14;
  
  /*if (Ramp_Motor.rotation(rotationUnits::deg) <= 400) {
    Ramp_Speed = 100;
  } else if (400 < Ramp_Motor.rotation(rotationUnits::deg) <= 500) {
    Ramp_Speed = 50;
  } else {
    Ramp_Speed = 10;
  }*/
  return Ramp_Speed;
}
// The Wheel's circumfrence is pi*4 inches
float inToRev(float inches) { return inches / (3.14159265358979323 * 4); }

void setLeftRightDriveSpeed(int leftSpeed = 0, int rightSpeed = 0) {
  FrontLeft_Motor.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  BackLeft_Motor.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  FrontRight_Motor.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
  BackRight_Motor.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
}
void driveForLeftRight(float leftRevs, float rightRevs, int speed) {
  FrontLeft_Motor.rotateFor(leftRevs, rotationUnits::rev, speed,
                            velocityUnits::pct, false);
  BackLeft_Motor.rotateFor(leftRevs, rotationUnits::rev, speed,
                           velocityUnits::pct, false);
  FrontRight_Motor.rotateFor(rightRevs, rotationUnits::rev, speed,
                             velocityUnits::pct, false);
  BackRight_Motor.rotateFor(rightRevs, rotationUnits::rev, speed,
                            velocityUnits::pct, true);
}
void swaysSpin(float leftSpeed, int rightSpeed = 0) {
  FrontLeft_Motor.spin(directionType::fwd, leftSpeed,
                            velocityUnits::pct);
  BackLeft_Motor.spin(directionType::rev, leftSpeed,
                           velocityUnits::pct);
  FrontRight_Motor.spin(directionType::rev, rightSpeed,
                             velocityUnits::pct);
  BackRight_Motor.spin(directionType::fwd, rightSpeed,
                            velocityUnits::pct);
}
void driveFor(float inches, int speed = 85) {
  FrontLeft_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                            velocityUnits::pct, false);
  BackLeft_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                           velocityUnits::pct, false);
  FrontRight_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                             velocityUnits::pct, false);
  BackRight_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                            velocityUnits::pct, true);
}
void sideways(float inches, int speed = 85) {
  FrontLeft_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                            velocityUnits::pct, false);
  BackLeft_Motor.rotateFor(-inToRev(inches), rotationUnits::rev, speed,
                           velocityUnits::pct, false);
  FrontRight_Motor.rotateFor(-inToRev(inches), rotationUnits::rev, speed,
                             velocityUnits::pct, false);
  BackRight_Motor.rotateFor(inToRev(inches), rotationUnits::rev, speed,
                            velocityUnits::pct, true);
}
void intakeoff(void) {
  Left_Intake_Motor.spin(directionType::fwd, 0, velocityUnits::pct);
  Right_Intake_Motor.spin(directionType::fwd, 0, velocityUnits::pct);
}
void initiate(void) {

  
  driveFor(5, 100);
  
}
void scoretower(void) {
  Ramp_Motor.rotateFor(-1.2, rotationUnits::rev, 100, velocityUnits::pct);
  Arm_Motor.rotateFor(2.8, rotationUnits::rev, 100, velocityUnits::pct);
}
void dfposition(void) {
  Arm_Motor.rotateFor(-2.8, rotationUnits::rev, 100, velocityUnits::pct);
  Ramp_Motor.rotateFor(1.2, rotationUnits::rev, 100, velocityUnits::pct);
}

void Intake(bool dir, int speed) {
  if (dir) {
    Left_Intake_Motor.spin(directionType::fwd, speed, velocityUnits::pct);
    Right_Intake_Motor.spin(directionType::fwd, speed, velocityUnits::pct);
  } else {
    Left_Intake_Motor.spin(directionType::rev, speed, velocityUnits::pct);
    Right_Intake_Motor.spin(directionType::rev, speed, velocityUnits::pct);
  }
}

int Raise_DR4B() {
  float degRamp = Ramp_Motor.rotation(rotationUnits::deg);
  float degDR4B = Arm_Motor.rotation(rotationUnits::deg);
  if (level_count == 1) {
    degRamp = 500;
    degDR4B = -650;
  } else if (level_count == 2) {
    degRamp = 600;
    degDR4B = -850;
  } else if (level_count == 3) {
    degRamp = 800;
    degDR4B = -1300;
  } else if (level_count == 0) {
    degRamp = 0;
    degDR4B = 0;
    
  }

  
  
  Arm_Motor.rotateTo(degDR4B, rotationUnits::deg, 100, velocityUnits::pct,
                     false);

  if(Ramp_Motor.rotation(rotationUnits::deg) >= degRamp){
    vex::this_thread::sleep_for( 300 );
  }
  Ramp_Motor.rotateTo(degRamp, rotationUnits::deg, 100,
                      velocityUnits::pct, true);
  button_switch = true;

  return 0;
}

void stack(void) {
  //Ramp_Motor.rotateTo(600, rotationUnits::deg, 100, velocityUnits::pct,true);

  //Ramp_Motor.rotateTo(800, rotationUnits::deg, 60, velocityUnits::pct,true);
 // Ramp_Motor.rotateTo(1000, rotationUnits::deg, 20, velocityUnits::pct,true);
  Left_Intake_Motor.stop(brakeType::coast);
  Right_Intake_Motor.stop(brakeType::coast);
  while(Ramp_Motor.rotation(rotationUnits::deg)<1020){
  Ramp_Motor.spin(directionType::fwd, Ramp_power(), velocityUnits::pct);
  }
  Ramp_Motor.stop(brakeType::hold);

  Intake(1, 90);
  task::sleep(100);
  driveFor(-10, 10);
  intakeoff();
}


//void scoretower(bool height) {}


#pragma region "Sonar code"
//Stops when reaches teh correct distance

//for Rotation
int SonarRotation() {
  float maxPower = 55;
  float minPower = 3;
  float kp = 2;
  float error = 0;
  int ticksElapsed = 0;
  int ticksTime = 0;
  int Max_Time = 50;//is not in millisecond, it is how many loops it runs.
  // Real time(millisecond) should be "Max_Time*(sleep time in loop)".
  float drivePower = 0;

  while (ticksElapsed < 3 && ticksTime < Max_Time) {
    error = RangeFinderG.distance(inches) - RangeFinderE.distance(inches);
  

    drivePower = error * kp;
    drivePower = capMinMax(drivePower, minPower, maxPower);
    //setLeftRightDriveSpeed(-drivePower, drivePower);
    speed_L = -drivePower;
    speed_R = drivePower;

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(error);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print(RangeFinderE.distance(inches));
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print(RangeFinderG.distance(inches));


    if(ticksTime >= Max_Time){
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Time out !!!");
    }
    vex::this_thread::sleep_for( 20 );
    

    if (abs(drivePower) < 1){
      ticksElapsed++;
    }else{
      ticksElapsed = 0;
    }

    ticksTime++;
    //ticksElapsed++;
  }
  speed_L = 0;
  speed_R = 0;
  
  //setLeftRightDriveSpeed(0, 0);
  return 0;
}
#pragma endregion

#pragma region "vision code"

int getVisionObjX() {
  // if (colour == PURPLE)
  Vision.takeSnapshot(SIG_1);
  // else
  // Vision.takeSnapshot(SIG_2);

  int objCount = Vision.objectCount;

  // Find the object closest to the center.
  // driveFor(2,90);
  // int closestDistance = 30000;
  int closestDistance = 0;
  int biggest_object = 0;
  for (int i = 0; i < objCount; i++) {
    if (Vision.objects[i].width > 4 &&
        Vision.objects[i].height > 4) { // Ignore tiny objects
      // Find distance from center
      // int disFromCenter = (VISION_WIDTH / 2) - Vision.objects[i].centerX;
      int disFromCenter = Vision.objects[i].width * Vision.objects[i].height;
      if (abs(disFromCenter) > abs(closestDistance)) {
        closestDistance = disFromCenter;
        biggest_object = i;
      }
    }

    // break;
  }
  closestDistance = (VISION_WIDTH / 2) - Vision.objects[biggest_object].centerX;

  // Controller.Screen.print(closestDistance);
  wait(20, msec); // Sleep the task for a short amount of time to
  // Controller.Screen.clearLine(); // prevent wasted resources.
  // if (i > 10)
  return closestDistance;
}

// This is the camera code.
int aimRobot() {
  // initialize P loop variables
  float maxPower = 50;
  float minPower = 2;
  float kp = .10;         // this is the kp. Change this down to make

  // Init timer stuff.
  int ticksElapsed = 0;
  float error = 0;
  float drivePower = 0;
  // finish check variables
  int ticksTime = 0;
  int Max_Time = 100;

  // run motors until target is within n px certainty
  while (ticksElapsed < 3 && ticksTime < Max_Time) {
    // Find distance from center.

    int disFromCenter = getVisionObjX();
    error = disFromCenter - VISION_OFFSET;

    // This means robot didn't find a target; exits
    /*if (disFromCenter <= VISION_MAX_DISTANCE) {
      atTarget = true;
      Brain.Screen.print("LOG: Vis-ensor failed to find object");
      Brain.Screen.newLine();
    }*/

    // Brain.Screen.print(closestDistance);
    drivePower = error * kp;
    drivePower = capMinMax(drivePower, minPower, maxPower);
    setLeftRightDriveSpeed(drivePower, -drivePower);

    // check for finish
    if (abs(error) < 3){
      ticksElapsed++;
    }else{
      ticksElapsed = 0;
    }

    ticksTime++;

    if(ticksTime >= Max_Time){
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Time out !!!");
    }
    vex::this_thread::sleep_for( HZ_15_WAIT );

  }
  setLeftRightDriveSpeed(0, 0);
  return 0;
}


int SwaysRobot() {
// initialize P loop variables
  float maxPower = 50;
  float minPower = 2;
  float kp = .4;         // this is the kp. Change this down to make

  // Init timer stuff.
  int ticksElapsed = 0;
  float error = 0;
  float drivePower = 0;
  // finish check variables
  int ticksTime = 0;
  int Max_Time = 3;

  // run motors until target is within n px certainty
  while (ticksElapsed < 3 && ticksTime < Max_Time) {
    // Find distance from center.

    int disFromCenter = getVisionObjX();
    error = disFromCenter - VISION_OFFSET;

    // This means robot didn't find a target; exits
    /*if (disFromCenter <= VISION_MAX_DISTANCE) {
      atTarget = true;
      Brain.Screen.print("LOG: Vis-ensor failed to find object");
      Brain.Screen.newLine();
    }*/

    // Brain.Screen.print(closestDistance);
    drivePower = error * kp;
    drivePower = capMinMax(drivePower, minPower, maxPower);
    swaysSpin(drivePower, drivePower);

    // check for finish
    if (abs(error) < 3){
      ticksElapsed++;
    }else{
      ticksElapsed = 0;
    }

    ticksTime++;

    if(ticksTime >= Max_Time){
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      Controller.Screen.print("Time out !!!");
    }
    vex::this_thread::sleep_for( HZ_15_WAIT );
  }
  setLeftRightDriveSpeed(0, 0);
  return 0;
}

#pragma endregion



void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}



void sonar_sideways(int Time, float speed){
int count = 0;
  while(count<Time){
    float greatest_speed = 0;

    float sonar_speed = (RangeFinderG.distance(inches) - RangeFinderE.distance(inches))*20;


    float motor_speed[] = {speed-sonar_speed,-speed-sonar_speed,-speed+sonar_speed,speed+sonar_speed};

    for(int i = 0; i < 4; i++) {
      if (motor_speed[i]>100){
        greatest_speed = motor_speed[i];
      }
    }
    if (greatest_speed>100){

      float rate = 100/greatest_speed;
      for(int i = 0; i < 4; i++) {
        motor_speed[i] = motor_speed[i]*rate;
      }
    }


    FrontLeft_Motor.spin(directionType::fwd, motor_speed[0], velocityUnits::pct);
    BackLeft_Motor.spin(directionType::fwd, motor_speed[1], velocityUnits::pct);
    FrontRight_Motor.spin(directionType::fwd, motor_speed[2], velocityUnits::pct);
    BackRight_Motor.spin(directionType::fwd, motor_speed[3], velocityUnits::pct);

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(speed_L);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print(speed_R);
    task::sleep( 20 );
    

   count++;
   }
   FrontLeft_Motor.stop();
    BackLeft_Motor.stop();
    FrontRight_Motor.stop();
    BackRight_Motor.stop();
   }
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void combine(){

}






void autonomous(void) {
  //initiate();
  Intake(0, 100); 
  //SwaysRobot(PURPLE);
  //aimRobot(PURPLE);
  driveFor(33, 90);
  driveFor(20, 50);
  driveForLeftRight(-.3,.3, 100);
  driveFor(-37, 100);
  intakeoff();
  driveFor(-30,100);
  
  //sideways(49, 100);
   //thread myThread = thread(SwaysRobot);
   //thread sonar_thread = thread(SonarRotation);
   sonar_sideways(2,90);
  //sideways(7, 100);
  //aimRobot();
  SwaysRobot();
  
  Intake(0, 100); 
  driveFor(50, 60);
  //task::sleep(300);
  driveFor(-12, 100);
  intakeoff();
  driveFor(-40, 100);
  driveForLeftRight(1.33,-1.33, 50);
  driveFor(11, 40);
  swaysSpin(30,30);
  task::sleep(50);
  stack();
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Teleoperated Control                         */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // ..........................................................................
    /*There are two different wheel moving direction value(Move_N, Move_P),
    and they come from Controller Axis 3 and Axis 4. Axis 3 controls vertical,
    and Axis 4 is opposite in Move_N and Move_P because the FL and FR (or BL and
    BR) wheels rotate in different direction while they moves horizontal.*/
    double Move_Neg =
        (Controller.Axis3.value() - Controller.Axis4.value()) * MOVING_RATE;
    double Move_Pos =
        (Controller.Axis3.value() + Controller.Axis4.value()) * MOVING_RATE;
    // Rotate is the value from Controller Axis 1.
    double Rotate = Controller.Axis1.position(percentUnits::pct) * TURNING_RATE;

    // Velocity_list[] include FL, FR, BL, and BR motors' velocity.
    double Velocity_list[] = {Move_Pos + Rotate, Move_Neg - Rotate,
                              Move_Neg + Rotate, Move_Pos - Rotate};

    // To find the largest absolute value in Velocity_list.
    double Largest_Speed = 0;
    int i;
    for (i = 0; i < 4; i++) {
      /*if the absolute Velocity in the list is larger than the "Largest_Speed"
      before, it becomes the new "Largest_Speed"*/
      Largest_Speed = abs(int(Velocity_list[i])) > Largest_Speed
                          ? abs(int(Velocity_list[i]))
                          : Largest_Speed;
    }

    /*if Largest_Speed is over the max speed, all velocity in the list lower
    down by a rate to the Max_Speed.*/
    if (Largest_Speed > MAX_SPEED) {
      double speed_rate = MAX_SPEED / Largest_Speed;
      for (i = 0; i < 4; i++) {
        Velocity_list[i] *= speed_rate;
      }
    }

    // Moter spin with the calculated velocity in the list.
    FrontLeft_Motor.spin(directionType::fwd, Velocity_list[0],
                         velocityUnits::pct);
    FrontRight_Motor.spin(directionType::fwd, Velocity_list[1],
                          velocityUnits::pct);
    BackLeft_Motor.spin(directionType::fwd, Velocity_list[2],
                        velocityUnits::pct);
    BackRight_Motor.spin(directionType::fwd, Velocity_list[3],
                         velocityUnits::pct);

    if (Controller.ButtonL2.pressing()) {
      Ramp_Motor.spin(directionType::fwd, Ramp_power(), velocityUnits::pct);
      button_switch = true;
    } else if (Controller.ButtonL1.pressing()) {
      Ramp_Motor.spin(directionType::rev, 100, velocityUnits::pct);
      button_switch = true;
    } else if(button_switch == true){
      if (Ramp_Motor.rotation(rotationUnits::deg) >= 400) {
        Ramp_Motor.stop(brakeType::hold);
      } else {
        Ramp_Motor.stop(brakeType::coast);
      }
    }

  if (Controller.ButtonLeft.pressing()){
stack();
  }

    if (Controller.ButtonRight.pressing()) {
      aimRobot();
      //SonarRotation();
    }

  
    if (Controller.ButtonA.pressing()) {
      if(button_switch == true){
      button_switch = false;
      level_count += 1;
      if (level_count > 3) {
        level_count = 0;
      }
      thread myThread = thread(Raise_DR4B);
      }
    }else if (Controller.ButtonY.pressing()) {
      if(button_switch == true){
      button_switch = false;
      level_count = 0;
      if (level_count < 0) {
        level_count = 3;
      }
      thread myThread = thread(Raise_DR4B);
      }
    }


    // ..........................................................................
    if (Controller.ButtonB.pressing()) {
      Arm_Motor.spin(directionType::fwd, 100, velocityUnits::pct); // up
      button_switch = true;
    } else if (Controller.ButtonX.pressing()) {
      Arm_Motor.spin(directionType::rev, 100, velocityUnits::pct); // down
      button_switch = true;
    } else if(button_switch == true){
      Arm_Motor.stop(brakeType::hold);
    }

    // ..........................................................................
    if (Controller.ButtonR2.pressing()) {
      Left_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
      Right_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller.ButtonR1.pressing()) {
      Left_Intake_Motor.spin(directionType::rev, 100, velocityUnits::pct);
      Right_Intake_Motor.spin(directionType::rev, 100, velocityUnits::pct);
    } else{
      if (Ramp_Motor.rotation(rotationUnits::deg) <= 1000) {
        Left_Intake_Motor.stop(brakeType::hold);
        Right_Intake_Motor.stop(brakeType::hold);
      } else {
        Left_Intake_Motor.stop(brakeType::coast);
        Right_Intake_Motor.stop(brakeType::coast);
      }
    }

    if (Controller.ButtonUp.pressing()) {
      FrontLeft_Motor.spin(directionType::fwd, 10, velocityUnits::pct);
      BackLeft_Motor.spin(directionType::fwd, 10, velocityUnits::pct);
      FrontRight_Motor.spin(directionType::fwd, 10, velocityUnits::pct);
      BackRight_Motor.spin(directionType::fwd, 10, velocityUnits::pct);
    } else if (Controller.ButtonDown.pressing()) {
      FrontLeft_Motor.spin(directionType::rev, 10, velocityUnits::pct);
      BackLeft_Motor.spin(directionType::rev, 10, velocityUnits::pct);
      FrontRight_Motor.spin(directionType::rev, 10, velocityUnits::pct);
      BackRight_Motor.spin(directionType::rev, 10, velocityUnits::pct);
    }

    Controller.Screen.setCursor(1,1);
    Controller.Screen.print(level_count);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print(Arm_Motor.rotation(rotationUnits::deg));



    Brain.Screen.print(RangeFinderE.distance(inches));
    //Brain.Screen.print(RangeFinderG.distance(inches));
    wait(20, msec); // Sleep the task for a short amount of time to
    Brain.Screen.clearLine(); // prevent wasted resources.


    Controller.Screen.clearScreen();
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }

}
