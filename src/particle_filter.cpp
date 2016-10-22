#include <algorithm>

#include <utils.h>
#include <particle_filter.h>
using namespace std;

/*
   Perform particle filter algorithm
   Return the estimated pose over time
 */
vector<Pose> particle_filter(
    const Map& map,
    const vector<SensorMsg*> sensor_msgs,
    const int kParticles) {

  vector<Pose> poses;

  // 1) Initialize particles drawn from uniform distribution
  vector<Particle> particles(kParticles);

  vector<Measurement> simulated_measurements(
    kParticles, /* kParticles => k simulations*/
    Measurement(
      Laser::kBeamPerScan /* each simulation is 180 degree */
    )
  );

  // 2) Perform particle filter algorithm iteratively
  for (size_t i=1; i<sensor_msgs.size(); ++i) {
    printf("Processing %zu-th message\n", i);

    auto sensor_msg = sensor_msgs[i];
    auto u_t = sensor_msgs[i]->pose - sensor_msgs[i-1]->pose;
    // cout << "u[" << i << "] = " << u_t << endl;
    
    update_particles_through_motion_model(u_t, particles);

    if (sensor_msg->type() == SensorMsg::Odometry) {
      // Do nothing
    }
    else {
      Laser* laser = dynamic_cast<Laser*>(sensor_msg);
      // cout << (laser->ranges) << endl;
      auto models = sensor_model(laser->ranges);
      // cout << measurement << endl;
      
      for (size_t j=0; j<kParticles; ++j)
	simulate_laser_scan(simulated_measurements[j], particles[j], map);

      compute_likelihood(simulated_measurements, models);
    }
  }

  return poses;
}

/*
   Perform 2-D Bresenham ray-tracing on the map. Collect all the probabilities
   along the ray and return as an vector
 */
float simulate_laser_per_beam(
    const int x_start, const int y_start,
    const int dx, const int dy,
    const Map& map) {

  // TODO
  return 0;
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void simulate_laser_scan(Measurement& m, const Pose& pose, const Map& map) {
  // TODO
  for (size_t i=0; i<m.size(); ++i)
    m[i] = simulate_laser_per_beam(0, 0, 0, 0, map);

  // cout << "simulation done" << endl;
}

/*
   <Sensor Model>
   Turn a single int measurement into a probability distribution based on the 
   sensor model.
   1. Measurement Noise    - Gaussian distribution centered at measurement z
   2. Unexpected Obstacles - Exponential decay
   3. Random Measurement   - Uniform distribution
   4. Max range            - a peak at z_max
 */
PDF sensor_model_per_beam(int z) {
  PDF model(100, 0.1);
  // TODO

  return model;
}

/* 
   <Sensor PDF>
   Turn a laser beam (180 integers) into a vector of probability distributions
 */
vector<PDF> sensor_model(const vector<int> &z) {

  vector<PDF> models(z.size());

  for (size_t i=0; i<z.size(); ++i)
    models[i] = sensor_model_per_beam(z[i]);

  return models;
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> compute_likelihood(
    const vector<Measurement>& simulated_measurements,
    const vector<PDF>& models) {

  // TODO
  for (size_t i=0; i<simulated_measurements.size(); ++i) {
  }

  return vector<float>();
}

/*
   <Motion Model>
 */
void update_particles_through_motion_model(
    const Pose& delta,
    vector<Pose>& poses,
    const float kSigma) {

  // TODO
}
