#ifndef ODOM_H_
#define ODOM_H_

#include "main.h"

class Odom {
  
  // Distances from tracking wheels to tracking centre.
  double tl;
  double tr;
  double tb;

  // Diameter of tracking wheel.
  double d;
  const double ticksToInch = (M_PI * d) / 360;

  double prevLEncRaw;
  double prevREncRaw;
  double prevBEncRaw;

  double currentXIn;
  double currentYIn;
  double currentHeadingRad;

  public:
  Odom(double _tl, double _tr, double _tb, double _d);
  void updateOdom(int lEncRawVal, int rEncRawVal, int bEncRawVal);
  void resetOdom();

  double getX(); // Inches.
  double getY(); // Inches.
  double getHeading(); // Rad.
  double getHeadingDeg(); // Deg.
};

#endif // ODOM_H_
