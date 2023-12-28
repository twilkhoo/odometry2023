#include "odom.h"

Odom::Odom(double _tl, double _tr, double _tb, double _d): 
  tl{_tl}, tr{_tr}, tb{_tb}, d{_d} {}

void Odom::updateOdom(int lEncRawVal, int rEncRawVal, int bEncRawVal) {
  double deltaLIn = (lEncRawVal - prevLEncRaw) * ticksToInch;
  double deltaRIn = (rEncRawVal - prevREncRaw) * ticksToInch;
  double deltaBIn = (bEncRawVal - prevBEncRaw) * ticksToInch;

  // Update previous (raw) values.
  prevLEncRaw = lEncRawVal;
  prevREncRaw = rEncRawVal;
  prevBEncRaw = bEncRawVal;

  // Get the delta heading for this timeframe.
  // We compute R - L, so counterclockwise rotations are positive.
  double deltaHeadingRad = (deltaRIn - deltaLIn) / (tr + tl);
  double halfAngle = deltaHeadingRad / 2;
  double sinHalfAngle = sin(halfAngle);

  // Hypotenuse of the triangle formed by centre of circle, and robot's start/end positions.
  double hypotenuseRight;
  double hypotenuseBack;

  if (deltaHeadingRad) {
    double radiusRight = deltaRIn / deltaHeadingRad;
    hypotenuseRight = (radiusRight + tr) * sinHalfAngle * 2;

    double radiusBack = deltaBIn / deltaHeadingRad;
    hypotenuseBack = (radiusBack + tb) * sinHalfAngle * 2;
  } else {
      hypotenuseRight = deltaRIn;
      hypotenuseBack = deltaBIn;
  }

  double globalHalfAngle = currentHeadingRad + halfAngle;
  double sinGlobalHalfAngle = sin(globalHalfAngle);
  double cosGlobalHalfAngle = cos(globalHalfAngle);

  currentXIn += hypotenuseRight * sinGlobalHalfAngle;
  currentYIn += hypotenuseRight * cosGlobalHalfAngle;

  currentXIn += hypotenuseBack * cosGlobalHalfAngle; // cos(-x) = cos(x)
  currentYIn += hypotenuseBack * -sinGlobalHalfAngle; // sin(-x) = -sin(x)

  // currentHeadingRad += deltaHeadingRad;
  currentHeadingRad = (rEncRawVal - lEncRawVal) / ((tr+tl)/2);
}

void Odom::resetOdom() {
  prevLEncRaw = 0;
  prevREncRaw = 0;
  prevBEncRaw = 0;

  currentXIn = 0;
  currentYIn = 0;
  currentHeadingRad = 0;
}

double Odom::getX() {
  return currentXIn;
}

double Odom::getY() {
  return currentYIn;
}

double Odom::getHeading() {
  return currentHeadingRad;
}

double Odom::getHeadingDeg() {
  return currentHeadingRad * 180 / M_PI;
}


