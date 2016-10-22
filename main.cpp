#include <iostream>
#include <fstream>
#include <vector>
#include <utils.h>
#include <cmdparser.h>

#include <particle_filter.h>

using namespace std;

int main(int argc, char* argv[]) {

  CmdParser cmd(argc, argv);

  cmd.add("robot_data_log");
  
  cmd.addGroup("General options:")
    .add("--map-filename", "filename of map", "data/map/wean.dat")
    .add("--k-beam-per-scan", "number of beam per LASER scan", "180");

  cmd.addGroup("Particle Filter options:")
    .add("-n", "number of particles", "10000");

  cmd.addGroup("Sensor Model options:")
    .add("--weights", "weights of Gaussian, Exponential Decay, Uniform, Max Range", "1,1,1,1")
    .add("--sigma", "standard deviation of Gaussian model", "0.01")
    .add("--exp-decay", "exponential decay rate k in exp(-kt)", "2");

  cmd.addGroup("Example usage: ./particle_filter data/log/robotdata2.log");

  if (!cmd.isOptionLegal())
    cmd.showUsageAndExit();

  string robot_data = cmd[1];

  string map_fn = cmd["--map-filename"];
  Laser::kBeamPerScan = cmd["--k-beam-per-scan"];
  int kParticles = cmd["-n"];

  ParticleFilter::sensor_model_weights = splitAsFloat(cmd["--weights"], ',');
  ParticleFilter::sigma = cmd["--sigma"];
  ParticleFilter::exp_decay = cmd["--exp-decay"];

  cout
    << "sensor model weights: " << ParticleFilter::sensor_model_weights
    << "standard deviation  : " << ParticleFilter::sigma << endl
    << "exponential decay   : " << ParticleFilter::exp_decay << endl;

  // Load map
  Map map(map_fn);
  cout << map << endl;

  // Load data log
  vector<SensorMsg*> sensor_msgs = parse_robot_data(robot_data);
  /*
  for (auto& sensor_msg : sensor_msgs)
    cout << *sensor_msg << endl;
    */

  // Initialie particle filter
  ParticleFilter particle_filter(map, kParticles);

  // Use particle_filter to filter out the pose of robot
  auto poses = particle_filter(sensor_msgs);

  return 0;
}
