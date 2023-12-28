#include "odom.h"
#include "drive.h"
#include "globals.h"
#include "main.h"

// HELPER FUNCTIONS ///////////////////////////////////////////////////////////

// Converts degree input to radian output.
static double degToRad(double deg) { return deg * (M_PI / 180); }

// Converts radian input to degree output.
static double radToDeg(double rad) { return rad * (180 / M_PI); }

// Braking function, GLOBAL_DELAY is 250ms by default, we can edit this in
// globals.h.
static void brakeFunction() {
  setDriveBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  stopDrive();
  pros::delay(GLOBAL_DELAY);
  setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

// Computes desired heading given chassis state and desired X and Y points.
// Inputted X and Y points must be in inches.
// The outputted value will be in (-360, 360).
static double desiredHeading(std::shared_ptr<Odom> chassis, double desiredX,
                             double desiredY) {
  double currentX = chassis->getX();
  double currentY = chassis->getY();

  double deltaX = desiredX - currentX;
  double deltaY = desiredY - currentY;

  if (deltaY == 0) {
    return deltaX > 0 ? 90 : 270;
  }

  double degValue = radToDeg(atan(deltaX / deltaY));
  // at this point, degValue will be (-90, 90)

  if (deltaY < 0)
    degValue += 180;
  if (degValue < 0)
    degValue += 360;

  return degValue;
}

// Computes the difference in angle of where it the robot is currently pointing
// vs where it needs to be pointing given X and Y coordinates. X and Y
// coordinates need to be in inches. The outputted value will be in (-180, 180).
static double deltaHeading(std::shared_ptr<Odom> chassis, double desiredX,
                           double desiredY) {
  double rawHeading = chassis->getHeadingDeg();

  double angleDiff = desiredHeading(chassis, desiredX, desiredY) - rawHeading;

  angleDiff = fmod(angleDiff, 360);
  if (angleDiff > 180)
    angleDiff -= 360;
  if (angleDiff < -180)
    angleDiff += 360;

  return angleDiff;
}

// Computes cartesian distance between two points (in inches).
static double distanceBetween(std::shared_ptr<Odom> chassis, double desiredX,
                              double desiredY) {
  double currentX = chassis->getX();
  double currentY = chassis->getY();

  double deltaX = desiredX - currentX;
  double deltaY = desiredY - currentY;

  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

// ODOMETRY TURNING FUNCTIONS /////////////////////////////////////////////////
// see motionalgos.h.
void turnToHeading(std::shared_ptr<Odom> chassis, double desiredHeading,
                   int speed, bool brake = true) {
  double rawCurrentHeading = chassis->getHeadingDeg();
  double processedHeading =
      fmod(rawCurrentHeading, 360); // processedHeading is now (-360, 360)
  if (processedHeading < 0)
    processedHeading += 360; // processedHeading is now [0, 360)

  bool turnRight;
  if (desiredHeading > 180) {
    turnRight = ((processedHeading > fmod((180 + desiredHeading), 360) &&
                  processedHeading < desiredHeading));
  } else {
    turnRight = ((processedHeading > (180 + desiredHeading) ||
                  processedHeading < desiredHeading));
  }
  // Now we know whether to turn right or left.

  if (turnRight) {
    double startingHeading = rawCurrentHeading;

    double targetAmount = desiredHeading - processedHeading;
    if (targetAmount < 0)
      targetAmount += 360;

    double amountSoFar = 0;

    while (rawCurrentHeading < startingHeading + targetAmount) {
      setDrive(speed, -speed);
      rawCurrentHeading = chassis->getHeadingDeg();
      pros::delay(20);
    }
  }

  else {
    double startingHeading = rawCurrentHeading;

    double targetAmount = processedHeading - desiredHeading;
    if (targetAmount < 0)
      targetAmount += 360;

    double amountSoFar = 0;

    while (rawCurrentHeading < startingHeading + targetAmount) {
      setDrive(-speed, speed);
      rawCurrentHeading = chassis->getHeadingDeg();
      pros::delay(20);
    }
  }

  brake ? brakeFunction() : stopDrive();
}

// see motionalgos.h.
void turnToHeadingPID(std::shared_ptr<Odom> chassis, double desiredHeading,
                      int speed, double turnkP = 0.05, bool brake = true) {
  // Implementation only using P controller (for now).

  double rawCurrentHeading = chassis->getHeadingDeg();
  double processedHeading =
      fmod(rawCurrentHeading, 360); // processedHeading is now (-360, 360)
  if (processedHeading < 0)
    processedHeading += 360; // processedHeading is now [0, 360)

  bool turnRight;
  if (desiredHeading > 180) {
    turnRight = ((processedHeading > fmod((180 + desiredHeading), 360) &&
                  processedHeading < desiredHeading));
  } else {
    turnRight = ((processedHeading > (180 + desiredHeading) ||
                  processedHeading < desiredHeading));
  }
  // now you know whether to turn right or left

  if (turnRight) {
    double startingHeading = rawCurrentHeading;

    double targetAmount = desiredHeading - processedHeading;
    if (targetAmount < 0)
      targetAmount += 360;
    double target = startingHeading + targetAmount;

    do {
      rawCurrentHeading = chassis->getHeadingDeg();
      double turnError = target - rawCurrentHeading;
      double pidPower = (turnError * turnkP);
      double finalSpeed = speed * pidPower;
      if (fabs(finalSpeed) > abs(speed))
        finalSpeed = speed;
      printf("finalSpeed: %f\n", finalSpeed);
      setDrive(finalSpeed, -finalSpeed);
      pros::delay(20);

    } while (rawCurrentHeading < target);
  }

  else {
    double startingHeading = rawCurrentHeading;

    double targetAmount = processedHeading - desiredHeading;
    if (targetAmount < 0)
      targetAmount += 360;
    double target = startingHeading - targetAmount;

    do {
      rawCurrentHeading = chassis->getHeadingDeg();
      double turnError = rawCurrentHeading - target;
      double pidPower = (turnError * turnkP);
      double finalSpeed = speed * pidPower;
      if (fabs(finalSpeed) > abs(speed))
        finalSpeed = speed;
      printf("finalSpeed: %f\n", finalSpeed);
      setDrive(-finalSpeed, finalSpeed);
      pros::delay(20);

    } while (rawCurrentHeading > target);
  }

  brake ? brakeFunction() : stopDrive();
}

// see motionalgos.h.
void turnToFacePoint(std::shared_ptr<Odom> chassis, double desiredX,
                     double desiredY, int speed, bool brake = true) {
  double requiredAngle = desiredHeading(chassis, desiredX, desiredY);
  turnToHeading(chassis, requiredAngle, speed, brake);
}

// ODOMETRY DRIVING FUNCTIONS /////////////////////////////////////////////////

// see motionalgos.h.
void driveStraight(double desiredDistance, int speed, bool brake = true,
                   double diameter = 2.75) {

  bool forward = desiredDistance > 0;
  double oneRev = M_PI * diameter;
  double requiredEncoderDistance = (desiredDistance / oneRev) * 360;
  double desiredEncoderValue =
      encoderLeft.get_value() + requiredEncoderDistance;

  if (forward) {
    while (encoderLeft.get_value() < desiredEncoderValue) {
      setDrive(speed, speed);
      pros::delay(20);
    }
  } else {
    while (encoderLeft.get_value() > desiredEncoderValue) {
      setDrive(-speed, -speed);
      pros::delay(20);
    }
  }

  brake ? brakeFunction() : stopDrive();
}

// see motionalgos.h.
void driveStraightWhileMaintainingHeading(std::shared_ptr<Odom> chassis,
                                          double desiredDistance, int speed,
                                          bool brake = true,
                                          double diameter = 2.75) {

  bool forward = desiredDistance > 0;
  double oneRev = M_PI * diameter;
  double requiredEncoderDistance = (desiredDistance / oneRev) * 360;
  double desiredEncoderValue =
      encoderLeft.get_value() + requiredEncoderDistance;

  double initHeading = chassis->getHeadingDeg();
  double currentHeading = initHeading;
  double angleConstant = 2.01; // basically a kP value for a PID control

  if (forward) {
    while (encoderLeft.get_value() < desiredEncoderValue) {
      currentHeading = chassis->getHeadingDeg();

      setDrive(speed + ((initHeading - currentHeading) * angleConstant),
               speed - ((initHeading - currentHeading) * angleConstant));
      pros::delay(20);
    }
  } else {
    while (encoderLeft.get_value() > desiredEncoderValue) {
      currentHeading = chassis->getHeadingDeg();

      setDrive(-(speed - ((initHeading - currentHeading) * angleConstant)),
               -(speed + ((initHeading - currentHeading) * angleConstant)));
      pros::delay(20);
    }
  }

  brake ? brakeFunction() : stopDrive();
}

// see motionalgos.h.
void driveToPointWhileMaintainingHeading(std::shared_ptr<Odom> chassis,
                                         double desiredX, double desiredY,
                                         int speed, bool brake = true) {

  pros::lcd::clear();
  double initDeltaHeading = deltaHeading(chassis, desiredX, desiredY);
  double angleThreshold = 60;
  double maxDist = 2;
  double angleFixConstant = 1.53; // basically the P in a PID controller

  while (distanceBetween(chassis, desiredX, desiredY) > maxDist) {

    double currentDeltaHeading = deltaHeading(chassis, desiredX, desiredY);
    // if the absolute difference between the initial deltaHeading and current
    // deltaHeading is ever more than the threshold, stop (you've probably
    // overshot)
    if (fabs(currentDeltaHeading - initDeltaHeading) > angleThreshold)
      break;

    driveLeftBack.move(speed + (currentDeltaHeading * angleFixConstant));
    driveLeftFront.move(speed + (currentDeltaHeading * angleFixConstant));
    driveRightBack.move(speed - (currentDeltaHeading * angleFixConstant));
    driveRightFront.move(speed - (currentDeltaHeading * angleFixConstant));

    pros::delay(20);
  }

  brake ? brakeFunction() : stopDrive();
}

// see motionalgos.h.
void aimAndDriveToPoint(std::shared_ptr<Odom> chassis, double desiredX,
                        double desiredY, int speed, int version,
                        bool brake = true) {
  turnToFacePoint(chassis, desiredX, desiredY, speed, brake);

  if (version == 1)
    driveStraight(distanceBetween(chassis, desiredX, desiredY), speed, brake);

  if (version == 2)
    driveStraightWhileMaintainingHeading(
        chassis, distanceBetween(chassis, desiredX, desiredY), speed, brake);

  if (version == 3)
    driveToPointWhileMaintainingHeading(chassis, desiredX, desiredY, speed,
                                        brake);
}
