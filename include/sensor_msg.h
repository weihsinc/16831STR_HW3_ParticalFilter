#ifndef __SENSOR_MSG_H_
#define __SENSOR_MSG_H_

#include <vector>
#include <string>

// typedef double FLOAT;
typedef float FLOAT;

class SensorMsg {

public:

  SensorMsg(const std::vector<std::string> &tokens) {
    // x, y, theta
    this->x = std::stof(tokens[1]);
    this->y = std::stof(tokens[2]);
    this->theta = std::stof(tokens[3]);

    // The last token is timestamp
    this->timestamp = std::stof(tokens.back());
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
  FLOAT x, y, theta;
  FLOAT timestamp;
};

class Odometry : public SensorMsg {
public:
  Odometry(const std::vector<std::string> &tokens) : SensorMsg(tokens) {
  }
  
  virtual std::ostream& print (std::ostream& os) const {
    os << "O " << x << " " << y << " " << theta << " " << timestamp;
  }

  Type type() const { return SensorMsg::Odometry; }
};

class Laser : public SensorMsg {
public:
  static constexpr int kBeamPerScan = 180;

  Laser(const std::vector<std::string> &tokens) : SensorMsg(tokens) {
      this->xl = std::stof(tokens[4]);
      this->yl = std::stof(tokens[5]);
      this->thetal = std::stof(tokens[6]);

      this->ranges.resize(kBeamPerScan);
      
      for (size_t i=0; i<kBeamPerScan; ++i)
	this->ranges[i] = stoi(tokens[7+i]);
  }

  virtual std::ostream& print (std::ostream& os) const {
    os << "L " << x << " " << y << " " << theta << " "
       << xl << " " << yl << " " << thetal << " ";

    for (auto& range : ranges)
      os << range << " ";
    os << timestamp;
    return os;
  }

  Type type() const { return SensorMsg::Laser; }

  /* Data Member */
  FLOAT xl, yl, thetal;
  std::vector<int> ranges;
};

#endif // __SENSOR_MSG_H_
