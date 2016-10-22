#ifndef __SENSOR_MSG_H_
#define __SENSOR_MSG_H_

#include <vector>
#include <string>

#include <pose.h>

typedef float TIME_T;

class SensorMsg {

public:

  SensorMsg(const std::vector<std::string> &tokens):
    pose(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])),
    timestamp(std::stof(tokens.back())) {
  }

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
  Odometry(const std::vector<std::string> &tokens) : SensorMsg(tokens) {
  }
  
  virtual std::ostream& print (std::ostream& os) const {
    os << "O " << pose.x << " " << pose.y << " " << pose.theta << " " << timestamp;
    return os;
  }

  Type type() const { return SensorMsg::Odometry; }
};

class Laser : public SensorMsg {
public:
  static constexpr int kBeamPerScan = 180;

  Laser(const std::vector<std::string> &tokens) :
    SensorMsg(tokens),
    pose_l(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])),
    ranges(kBeamPerScan) {

    for (size_t i=0; i<kBeamPerScan; ++i)
      this->ranges[i] = stoi(tokens[7+i]);
  }

  virtual std::ostream& print (std::ostream& os) const {
    os << "L "
      << pose.x << " " << pose.y << " " << pose.theta << " "
      << pose_l.x << " " << pose_l.y << " " << pose_l.theta << " ";

    for (auto& range : ranges)
      os << range << " ";
    os << timestamp;

    return os;
  }

  Type type() const { return SensorMsg::Laser; }

  /* Data Member */
  Pose pose_l;
  std::vector<int> ranges;
};

#endif // __SENSOR_MSG_H_
