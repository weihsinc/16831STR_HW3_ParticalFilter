#include <iostream>
#include <fstream>

#include <utils.h>
#include <data_parser.h>

using namespace std;

vector<SensorMsg*> parse_robot_data(const string& robotdata_log_fn) {
  vector<SensorMsg*> sensor_msgs;

  ifstream fin(robotdata_log_fn);
  string line;

  SensorMsg* sensor_msg;

  while (std::getline(fin, line)) {
    auto tokens = split(line);

    // The first token is type, let the constructor handle the rest
    if (tokens[0] == "O")
      sensor_msg = new Odometry(tokens);
    else if (tokens[0] == "L")
      sensor_msg = new Laser(tokens);

    sensor_msgs.push_back(sensor_msg);
  }

  /*
  printf("Successfully parsed \33[33m%zu\33[0m sensor messages\n",
      sensor_msgs.size());
      */

  fin.close();

  return sensor_msgs;
}
