#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_

#include <map.h>
#include <pose.h>
#include <sensor_msg.h>
#include <data_parser.h>

typedef std::vector<FLOAT> PDF;
typedef Pose Particle;

/*
   Perform particle filter algorithm
 */
std::vector<Pose> particle_filter(
    const Map& map,
    const std::vector<SensorMsg*> sensor_msgs,
    const int kParticles = 1000);

/*
   Perform 2-D Bresenham ray-tracing on the map. Collect all the probabilities
   along the ray and return as an vector
 */
float simulate_laser_per_beam(
    const int x_start, const int y_start,
    const int dx, const int dy,
    const Map& map);

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void simulate_laser_scan(Measurement& m, const Pose& pose, const Map& map);

/*
   Turn a single int measurement into a probability distribution based on the 
   sensor model.
 */
PDF sensor_model_per_beam(int z);

/* 
   Turn a laser beam (180 integers) into a vector of probability distributions
 */
std::vector<PDF> sensor_model(const std::vector<int> &z);

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
std::vector<FLOAT> compute_likelihood(
    const std::vector<Measurement>& simulated_measurements,
    const std::vector<PDF>& models);

/*
   Update particles through motion model assuming it's Gaussian distribution
 */
void update_particles_through_motion_model(
    const Pose& delta,
    std::vector<Pose>& poses,
    const FLOAT kSigma = 0.01);

#endif // __PARTICLE_FILTER_H_
