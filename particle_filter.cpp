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
    .add("--show-ray-tracing", "turn on/off ray tracing result", "false");

  cmd.addGroup("Particle Filter options:")
    .add("-n", "number of particles", "1000");

  cmd.addGroup("Motion Model options:")
    .add("--motion-sigma-xy", "standard deviation of xy in Gaussian Motion model", "5")
    .add("--motion-sigma-theta", "standard deviation of theta in Gaussian Motion model", "0.003490658503988659");

  cmd.addGroup("Sensor Model options:")
    .add("--max-range", "max range of the laser sensor")
    .add("--k-beam-per-scan", "number of beam per LASER scan", "180")
    .add("--weights", "weights of Gaussian, Exponential Decay, Uniform, Max Range", "1,0.2,100,1")
    .add("--sigma", "standard deviation of Gaussian model", "50")
    .add("--exp-decay", "exponential decay rate k in exp(-kt)", "1e-3");

  cmd.addGroup("Example usage: ./particle_filter data/log/robotdata2.log");

  if (!cmd.isOptionLegal())
    cmd.showUsageAndExit();

  string robot_data = cmd[1];

  string map_fn = cmd["--map-filename"];
  Laser::kBeamPerScan = cmd["--k-beam-per-scan"];
  Laser::MaxRange = cmd["--max-range"];
  int kParticles = cmd["-n"];

  ParticleFilter::show_ray_tracing = cmd["--show-ray-tracing"];

  SensorModel::weights = splitAsFloat(cmd["--weights"], ',');
  SensorModel::sigma = cmd["--sigma"];
  SensorModel::exp_decay = cmd["--exp-decay"];

  // Load map
  Map map(map_fn);

  // Load data log
  vector<SensorMsg*> sensor_msgs = parse_robot_data(robot_data);

  // basic OpenCV example for later debugging
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  if (ParticleFilter::show_ray_tracing) {
    cv::namedWindow("Ray-Tracing Simulation (Naive)", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Ray-Tracing Simulation (Bresenham)", cv::WINDOW_AUTOSIZE);
  }

  // Initialie motion model
  float sigma_xy = cmd["--motion-sigma-xy"];
  float sigma_theta = cmd["--motion-sigma-theta"];
  MotionModel motion_model(sigma_xy, sigma_xy, sigma_theta);

  // Print all configurations
  clog
    << "# Beams per scan    : " << Laser::kBeamPerScan << endl
    << "Laser Max Range     : " << Laser::MaxRange << endl
    << "motion sigma xy     : " << sigma_xy << endl
    << "motion sigma theta  : " << sigma_theta << endl
    << "sensor model weights: " << SensorModel::weights
    << "standard deviation  : " << SensorModel::sigma << endl
    << "exponential decay   : " << SensorModel::exp_decay << endl;

  clog << map << endl;

  // Initialie particle filter
  ParticleFilter particle_filter(map, kParticles, motion_model);

  // Use particle_filter to filter out the pose of robot
  auto poses = particle_filter(sensor_msgs);

  return 0;
}
