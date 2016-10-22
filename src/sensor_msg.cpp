#include <sensor_msg.h>
using namespace std;

/* Implementation of class SensorMsg */
SensorMsg::SensorMsg(const vector<string> &tokens):
  pose(stof(tokens[1]), stof(tokens[2]), stof(tokens[3])),
  timestamp(stof(tokens.back())) {
}

/* Implementation of class Odometry */
Odometry::Odometry(const vector<string> &tokens) :
  SensorMsg(tokens) {
}

ostream& Odometry::print(ostream& os) const {
  os << "O " << pose.x << " " << pose.y << " " << pose.theta << " " << timestamp;
  return os;
}

SensorMsg::Type Odometry::type() const {
  return SensorMsg::Odometry;
}

/* Implementation of class Laser */
int Laser::kBeamPerScan;
int Laser::MaxRange;

Laser::Laser(const vector<string> &tokens) :
  SensorMsg(tokens),
  pose_l(stof(tokens[1]), stof(tokens[2]), stof(tokens[3])),
  ranges(kBeamPerScan) {

  for (size_t i=0; i<kBeamPerScan; ++i)
    this->ranges[i] = stoi(tokens[7+i]);
}

ostream& Laser::print(ostream& os) const {
  os << "L "
    << pose.x << " " << pose.y << " " << pose.theta << " "
    << pose_l.x << " " << pose_l.y << " " << pose_l.theta << " ";

  for (auto& range : ranges)
    os << range << " ";
  os << timestamp;

  return os;
}

SensorMsg::Type Laser::type() const {
  return SensorMsg::Laser;
}
