#include <iostream>
#include <fstream>
#include <vector>

#include <particle_filter.h>

using namespace std;

int main(int argc, char* argv[]) {

  if (argc >= 2) {
    printf("Usage: %s robot_data_log\n", argv[0]);
    return -1;
  }

  string robot_data(argc == 2 ? argv[1] : "data/log/robotdata1.log");

  Map map("data/map/wean.dat");

  cout << map << endl;

  vector<SensorMsg*> sensor_msgs = parse_robot_data(robot_data);

  /*
  for (auto& sensor_msg : sensor_msgs)
    cout << *sensor_msg << endl;
    */

  auto poses = particle_filter(map, sensor_msgs, 1000);

  return 0;
}
