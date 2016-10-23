#ifndef __POSE_H_
#define __POSE_H_

#include <iostream>

// typedef double FLOAT;
typedef float FLOAT;

class Pose {
public:
  Pose(): x(0), y(0), theta(0) {}
  Pose(FLOAT x, FLOAT y, FLOAT theta): x(x), y(y), theta(theta) {}

  Pose& operator += (const Pose& rhs) {
    *this = *this + rhs;
    return *this;
  }

  Pose operator + (const Pose& rhs) const {
    return Pose(x + rhs.x, y + rhs.y, theta + rhs.theta);
  }

  Pose operator - (const Pose& rhs) const {
    return Pose(x - rhs.x, y - rhs.y, theta - rhs.theta);
  }

  Pose operator * (const FLOAT c) const {
    return Pose(c*x, c*y, c*theta);
  }

  Pose& operator /= (const FLOAT c) {
    *this = *this / c;
    return *this;
  }

  Pose operator / (const FLOAT c) const {
    return Pose(x/c, y/c, theta/c);
  }

  friend std::ostream& operator << (std::ostream& os, const Pose& pose) {
    os << pose.x << " " << pose.y << " " << pose.theta;
    return os;
  }

  // Data Member
  FLOAT x, y, theta;
};

#endif // __POSE_H_
