#include <iostream>
#include <fstream>
#include <vector>

#include <particle_filter.h>

using namespace std;

int main(int argc, char* argv[]) {

  Map map("data/map/wean.dat");

  cout << map << endl;

  vector<SensorMsg*> sensor_msgs = parse_robot_data("data/log/robotdata1.log");

  Pose pose = particle_filter(map, sensor_msgs, 10000);

  return 0;
}
