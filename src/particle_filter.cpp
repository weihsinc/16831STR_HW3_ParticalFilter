#include <particle_filter.h>
using namespace std;

/*
   Perform particle filter algorithm
 */
Pose particle_filter(
    const Map& map,
    const vector<SensorMsg*> sensor_msgs,
    const int nParticles) {

  Pose pose;

  // TODO
  // Perform particle filter, and return the average pose of all particles

  return pose;
}

/*
   Perform 2-D Bresenham ray-tracing on the map. Collect all the probabilities
   along the ray and return as an vector
 */
Prob simulate_laser_per_beam(
    const int x_start, const int y_start,
    const int dx, const int dy,
    const Map& map) {

  Prob prob;

  // TODO

  return prob;
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
Measurement simulate_laser_scan(const Pose& pose, const Map& map) {
  Measurement m;
  // TODO
  // for i = 1 : 180 ...

  return m;
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
Prob sensor_model_per_beam(int z) {

  Prob prob;

  return prob;
}

/* 
   <Sensor Model>
   Turn a laser beam (180 integers) into a vector of probability distributions
 */
Measurement sensor_model(vector<int> z) {

  Measurement m;

  return m;
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> compute_likelihood(
    const vector<Measurement>& simulated_measurements,
    const Measurement& measurement) {

  // TODO

  return vector<float>();
}

/*
   <Motion Model>
 */
void update_particles_through_motion_model(
    const Pose& delta, vector<Pose>& poses) {
  const float kSigma = 0.1;

  // TODO
}
