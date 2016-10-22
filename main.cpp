#include <iostream>
#include <fstream>
#include <vector>
#include <cmdparser.h>

#include <particle_filter.h>

using namespace std;

int main(int argc, char* argv[]) {

  CmdParser cmd(argc, argv);

  cmd.add("robot_data_log");
  
  cmd.addGroup("General options:")
    .add("--map-filename", "filename of map", "data/map/wean.dat")
    .add("--beam-per-scan", "number of beam per LASER scan", "180");

  cmd.addGroup("Particle Filter options:")
    .add("-n", "number of particles", "10000");

  cmd.addGroup("Sensor Model options:")
    .add("--weights", "weights of Gaussian, Exponential Decay, Uniform, Max Range", "1,1,1,1")
    .add("--std", "standard deviation of Gaussian model", "0.01")
    .add("--exp-decay", "exponential decay rate k in exp(-kt)", "2");

  cmd.addGroup("Example usage: ./particle_filter data/log/robotdata2.log");

  if (!cmd.isOptionLegal())
    cmd.showUsageAndExit();

  string robot_data = cmd[1];
  string map_fn = cmd["--map-filename"];

  // Load map
  Map map(map_fn);
  cout << map << endl;

  // Load data log
  vector<SensorMsg*> sensor_msgs = parse_robot_data(robot_data);
  /*
  for (auto& sensor_msg : sensor_msgs)
    cout << *sensor_msg << endl;
    */

  auto poses = particle_filter(map, sensor_msgs, 1000);

  return 0;
}
