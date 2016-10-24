#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_

#include <map.h>
#include <pose.h>
#include <sensor_msg.h>
#include <data_parser.h>
#include <sensor_model.h>
#include <thread>

typedef std::vector<FLOAT> PDF;
typedef Pose Particle;

class ParticleFilter {
public:
  static FLOAT motion_sigma;
  static bool show_ray_tracing;

  static std::random_device rd;
  static std::mt19937 gen;

  enum RayTracingAlgorithm {
    Naive,
    Bresenham
  };

  /*
     Constructor
   */
  ParticleFilter(
      const Map& map,
      const size_t kParticles,
      const size_t nThreads = 8);

  /*
     Perform particle filter algorithm
   */
  std::vector<Pose> operator () (const std::vector<SensorMsg*> sensor_msgs);

private:
  /*
     Random initialize particles uniformly
    */
  void init_particles();

  /*
     Perform 2-D Bresenham ray-tracing on the map.
   */
  int Bresenham_ray_tracing(
      const int x_start, const int y_start,
      const int dx, const int dy,
      bool debug = false);

  /*
     Perform 2-D naive ray-tracing on the map.
   */
  int naive_ray_tracing(
      const FLOAT x, const FLOAT y,
      const FLOAT dx, const FLOAT dy);

  /*
     Perform 2-D ray-tracing using either naive approach or Bresenham
    */
  int simulate_laser_beam(
      const FLOAT x, const FLOAT y,
      const FLOAT dx, const FLOAT dy,
      RayTracingAlgorithm rta = RayTracingAlgorithm::Naive);

  /*
     Perform 2-D ray-tracing for all 180 beams for all particles
   */
  void simulate_laser_scan_all_particles();

  void simulate_laser_scan_some_particles(size_t i_begin, size_t i_end);

  /*
     Perform 2-D ray-tracing for all 180 beams in a single laser scan
   */
  void simulate_laser_scan(Measurement& m, const Pose& pose);

  /*
     Compare Laser measurements (per scan, meaning 180 feature vectors) to the
     measurements simulated at all particle's pose.
     */
  std::vector<FLOAT> compute_likelihood(const Measurement& measurement);
  
  /*
     Low variance re-sampling
    */
  std::vector<Particle> low_variance_resampling(
      const std::vector<FLOAT>& weights);

  /*
     Update particles through motion model assuming it's Gaussian distribution
   */
  void update_particles_through_motion_model(
    const Pose& p0, const Pose& p1);
  
  /*
     Compute particle centroid (weighted sum)
    */
  Particle compute_particle_centroid(const std::vector<FLOAT>& weights) const;

  /*
     Show particles on map
   */
  void show_particles_on_map() const;

  // Data Member
  cv::Mat simulation_naive;
  cv::Mat simulation_bresenham;

  const Map& map;
  size_t kParticles;

  std::vector<Particle> particles;
  std::vector<Measurement> simulated_measurements;

  std::vector<std::thread> threads;
  std::vector<bool> particle_mask;
};


#endif // __PARTICLE_FILTER_H_
