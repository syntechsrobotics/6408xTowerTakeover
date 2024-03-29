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
vex::motor FrontLeft_Motor = vex::motor(vex::PORT4, false);
vex::motor FrontRight_Motor = vex::motor(vex::PORT3, true);
vex::motor BackLeft_Motor = vex::motor(vex::PORT2, false);
vex::motor BackRight_Motor = vex::motor(vex::PORT1, true);
vex::controller Controller = vex::controller();
vex::motor Ramp_Motor = vex::motor(vex::PORT14, true);
vex::motor Arm_Motor = vex::motor(vex::PORT19, true);
vex::motor Left_Intake_Motor = vex::motor(vex::PORT11, true);
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

const int PURPLE = 0;
const int GREEN = 1;
int level_count = 0;
int DR4B_LEVEL = 0;



// void LimitSwitchPressed() { Arm_Motor.stop(); }

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
  float Ramp_Speed;
  Ramp_Speed = 100 - Ramp_Motor.rotation(rotationUnits::deg) / 12;
  /*if (Ramp_Motor.rotation(rotationUnits::deg) <= 400) {
    Ramp_Speed = 100;
  } else if (400 < Ramp_Motor.rotation(rotationUnits::deg) <= 450) {
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
  FrontLeft_Motor.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  BackLeft_Motor.spin(directionType::rev, leftSpeed, velocityUnits::pct);
  FrontRight_Motor.spin(directionType::rev, rightSpeed, velocityUnits::pct);
  BackRight_Motor.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
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

  Left_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
  Right_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
  driveFor(5, 10);
  intakeoff();
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
    Left_Intake_Motor.spin(directionType::fwd, -90, velocityUnits::pct);
    Right_Intake_Motor.spin(directionType::fwd, -90, velocityUnits::pct);
  }
}
int Raise_DR4B( ) {

 int level=2;
   int raiseSpeed = 100;
  float degRamp = Ramp_Motor.rotation(rotationUnits::deg);
  float degDR4B = Arm_Motor.rotation(rotationUnits::deg);
  if (level == 1) {
    degRamp = 500;
    degDR4B = -650;
  } else if (level == 2) {
    degRamp = 600;
    degDR4B = -850;
  } else if (level == 3) {
    degRamp = 800;
    degDR4B = -1300;
  } else if (level == 0) {
    degRamp = 0;
    degDR4B = 0;
    raiseSpeed = 55;
  }
  Ramp_Motor.rotateTo(degRamp, rotationUnits::deg, raiseSpeed,
                      velocityUnits::pct, false);
  // task::sleep(100);
  Arm_Motor.rotateTo(degDR4B, rotationUnits::deg, 100, velocityUnits::pct,
                     true);
  return 0;
}
void stack(void) {
  Ramp_Motor.rotateFor(2.27, rotationUnits::rev, 100, velocityUnits::pct);
  Intake(1, 60);
  task::sleep(500);
  driveFor(-10, 20);
  intakeoff();
}

void scoretower(bool height) {}
#pragma region "Sonar rotation code"
void SonarRotation(float maxPower = 55, float minPower = 3) {
  float kp = 2;
  /*float sonar1val = ;
  float sonar2val = RangeFinderG.distance(inches);*/
  float error = 0;
  int ticksElapsed = 0;
  int timeTicks = 0;
  bool atTarget = false;
  //float targetReading = 0;
  float drivePower = 0;

  while (!atTarget) {
    error = RangeFinderE.distance(inches) - RangeFinderG.distance(inches);
  

    drivePower = error * kp;
    drivePower = capMinMax(drivePower, minPower, maxPower);
    setLeftRightDriveSpeed(-drivePower, drivePower);
    if (abs(error) <1 ) {
      //ticksElapsed = 0;
      atTarget = true;
    
    }
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print(error);
    //Controller.Screen.print(drivePower);
    //Controller.Screen.clearLine();
    //Controller.Screen.newLine();
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print(RangeFinderE.distance(inches));
    //Controller.Screen.newLine();
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print(RangeFinderG.distance(inches));
    task::sleep(20);
    Controller.Screen.clearScreen();
/*
    if (ticksElapsed > 4) {
      atTarget = true;
       
    }*/
   
   // ticksElapsed++;
    //timeTicks++;
  }
  setLeftRightDriveSpeed(0, 0);
}
#pragma endregion

#pragma region "vision code"

int getVisionObjX(int colour) {
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
void aimRobot(int colour) {
  // initialize P loop variables
  float maxPower = 50;
  float minPower = 2;
  int timeOut = 4000 / 67; // 4000 / 67;
  float kp = .10;          // this is the kp. Change this down to make

  // Init timer stuff.
  int ticksElapsed = 0;
  int timeTicks = 0;
  float error = 0;
  float drivePower = 0;
  // finish check variables
  bool atTarget = false;

  // run motors until target is within n px certainty
  while (!atTarget || (timeTicks < timeOut)) {
    // Find distance from center.

    int disFromCenter = getVisionObjX(colour);
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
    if (abs(error) > 9) {
      ticksElapsed = 0;
    }
    if (ticksElapsed > 3) {
      atTarget = true;
    }
    ticksElapsed++;
    timeTicks++;
    task::sleep(HZ_15_WAIT);
  }
  setLeftRightDriveSpeed(0, 0);
}

void SwaysRobot(int colour) {
  // initialize P loop variables
  float maxPower = 50;
  float minPower = 4;
  int timeOut = 59; // 4000 / 67;
  float kp = .3;    // this is the kp. Change this down to make

  // Init timer stuff.
  int ticksElapsed = 0;
  int timeTicks = 0;
  float error = 0;
  float drivePower = 0;
  // finish check variables
  bool atTarget = false;

  // run motors until target is within n px certainty
  while (!atTarget || (timeTicks < timeOut)) {
    // Find distance from center.

    int disFromCenter = getVisionObjX(colour);
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
    if (abs(error) > 9) {
      ticksElapsed = 0;
    }
    if (ticksElapsed > 3) {
      atTarget = true;
    }
    ticksElapsed++;
    timeTicks++;
    task::sleep(HZ_15_WAIT);
  }
  setLeftRightDriveSpeed(0, 0);
}

#pragma endregion

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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

void autonomous(void) {
  /*initiate();

  aimRobot(PURPLE);
  driveFor(20, 70);
  driveFor(-20, 40);
  task::sleep(500); // goes for the first set of boxes

  driveFor(-35, 70);
  task::sleep(500);
   // retreats*/

  // driveFor(15, 49);
  SwaysRobot(PURPLE);
  /*Intake(0, 45);
  driveFor(40, 40);
  task::sleep(300);
  intakeoff();
  driveFor(-38, 40);

  driveForLeftRight(1.7,-1.7, 40);
  sideways(29, 70);*/
  // driveFor(7, 70);
  // stack();
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
    } else if (Controller.ButtonL1.pressing()) {
      Ramp_Motor.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      if (Ramp_Motor.rotation(rotationUnits::deg) >= 400) {
        Ramp_Motor.stop(brakeType::hold);
      } else {
        Ramp_Motor.stop(brakeType::coast);
      }
    }
    if (Controller.ButtonRight.pressing()) {
      SonarRotation();
    }
    if (Controller.ButtonA.pressing()) {

      level_count += 1;
      if (level_count > 3) {
        level_count = 0;
      }
      thread myThread = thread(Raise_DR4B);
    }
    if (Controller.ButtonY.pressing()) {

      level_count = 0;
      if (level_count < 0) {
        level_count = 3;
      }
      ///Raise_DR4B(level_count);
    
      
    }

    // ..........................................................................
    if (Controller.ButtonB.pressing()) {
      Arm_Motor.spin(directionType::fwd, 100, velocityUnits::pct); // up
    } else if (Controller.ButtonX.pressing()) {
      Arm_Motor.spin(directionType::rev, 100, velocityUnits::pct); // down
    } else {
      Arm_Motor.stop(brakeType::hold);
    }

    // ..........................................................................
    if (Controller.ButtonR2.pressing()) {
      Left_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
      Right_Intake_Motor.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller.ButtonR1.pressing()) {
      Left_Intake_Motor.spin(directionType::rev, 100, velocityUnits::pct);
      Right_Intake_Motor.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      Left_Intake_Motor.stop(brakeType::hold);
      Right_Intake_Motor.stop(brakeType::hold);
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
    /*Controller.Screen.newLine();
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print(Ramp_Motor.rotation(rotationUnits::deg));
    Controller.Screen.newLine();
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print(Ramp_power());*/
    // Brain.Screen.print(RangeFinderG.distance(inches));
    wait(20, msec); // Sleep the task for a short amount of time to
    Controller.Screen.clearLine(); // prevent wasted resources.
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
