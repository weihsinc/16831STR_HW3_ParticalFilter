#ifndef __SENSOR_MSG_H_
#define __SENSOR_MSG_H_

#include <vector>
#include <string>

#include <pose.h>

typedef float TIME_T;
typedef std::vector<int> Measurement;

class SensorMsg {

public:

  SensorMsg(const std::vector<std::string> &tokens);

  enum Type {
    Odometry, Laser
  };

  friend std::ostream& operator << (std::ostream& os, const SensorMsg& msg) {
    return msg.print(os);
  }

  virtual Type type() const = 0;
  virtual std::ostream& print (std::ostream& os) const = 0;

  /* Data Member */
  Pose pose;
  TIME_T timestamp;
};

class Odometry : public SensorMsg {
public:
  Odometry(const std::vector<std::string> &tokens);
  
  virtual std::ostream& print (std::ostream& os) const;

  Type type() const;
};

class Laser : public SensorMsg {
public:
  static int kBeamPerScan;
  static int MaxRange;

  Laser(const std::vector<std::string> &tokens);

  virtual std::ostream& print (std::ostream& os) const;

  Type type() const;

  /* Data Member */
  Pose pose_l;
  Measurement ranges;
};

#endif // __SENSOR_MSG_H_
