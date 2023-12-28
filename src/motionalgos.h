#ifndef ODOMFUNCS_H
#define ODOMFUNCS_H

#include "odom.h"
#include "main.h"


// Turn to a specific heading (deg).
extern void turnToHeading(std::shared_ptr<Odom> chassis, double desiredHeading,
                          int speed, bool brake = true);

// Turn to a specific heading using PID control.
extern void turnToHeadingPID(std::shared_ptr<Odom> chassis,
                             double desiredHeading, int speed,
                             double turnkP = 0.05, bool brake = true);

// Turn to face a specific point.
extern void turnToFacePoint(std::shared_ptr<Odom> chassis, double desiredX,
                            double desiredY, int speed, bool brake = true);

// Drive straight (forward by default, backward=>negative distance) for a
// certain distance (inches).
extern void driveStraight(double desiredDistance, int speed, bool brake = true,
                          double diameter = 2.75);

// Drive straight while maintaining heading, using Odometry control to account
// for disturbances in heading.
extern void driveStraightWhileMaintainingHeading(std::shared_ptr<Odom> chassis,
                                                 double desiredDistance,
                                                 int speed, bool brake = true,
                                                 double diameter = 2.75);

// Drive to a certain point while maintaining heading, assuming that the robot
// already (almost) faces that point.
extern void driveToPointWhileMaintainingHeading(std::shared_ptr<Odom> chassis,
                                                double desiredX,
                                                double desiredY, int speed,
                                                bool brake = true);

// First turn to face a point, then drive to the point.
extern void aimAndDriveToPoint(std::shared_ptr<Odom> chassis, double desiredX,
                               double desiredY, int speed, int version,
                               bool brake = true);

#endif