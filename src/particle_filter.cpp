#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>

#include <utils.h>
#include <particle_filter.h>
using namespace std;

FLOAT ParticleFilter::motion_sigma;

std::random_device ParticleFilter::rd;
std::mt19937 ParticleFilter::gen(rd());

ParticleFilter::ParticleFilter(const Map& map, const int kParticles):
  map(map), kParticles(kParticles) {
}

constexpr FLOAT PI() { return std::atan(1) * 4; }

/*
   Perform particle filter algorithm
   Return the estimated pose over time
 */
vector<Pose> ParticleFilter::operator () (const vector<SensorMsg*> sensor_msgs) {

  vector<Pose> poses;

  // 1) Initialize particles drawn from uniform distribution
  auto particles = init_particles();

  // Pre-allocate all memory needed to store simulation results
  vector<Measurement> simulated_measurements(
    kParticles, /* kParticles => k simulations*/
    Measurement(
      Laser::kBeamPerScan /* each simulation is 180 degree */
    )
  );

  // Pre-allocate all memory needed to store sensor model
  vector<PDF> models(kParticles, PDF(Laser::MaxRange));

  // 2) Perform particle filter algorithm iteratively
  for (size_t i=1; i<sensor_msgs.size(); ++i) {
    // printf("Processing %zu-th message\n", i);

    auto sensor_msg = sensor_msgs[i];
    auto u_t = sensor_msgs[i]->pose - sensor_msgs[i-1]->pose;
    // cout << "u[" << i << "] = " << u_t << endl;

    show_particles_on_map(particles);

    update_particles_through_motion_model(u_t, particles);

    if (sensor_msg->type() == SensorMsg::Odometry) {
      // Do nothing
    }
    else {
      Laser* laser = dynamic_cast<Laser*>(sensor_msg);

      // compute_sensor_model(models, laser->ranges);
      
      // simulation = map.cv_img.clone();
      for (size_t j=0; j<kParticles; ++j)
	simulate_laser_scan(simulated_measurements[j], particles[j], map);
      // cv::imshow("Simulation", simulation);

      auto likelihoods = compute_likelihood(simulated_measurements, laser->ranges);

      for (size_t j=0; j<particles.size(); ++j) {
	if (!map.inside(particles[j]))
	  likelihoods[j] *= 0;
      }

      auto centroid = compute_particle_centroid(particles, likelihoods);

      cout << "pose[" << i << "] = " << centroid << endl;

      particles = low_variance_resampling(particles, likelihoods);
    }

    cv::waitKey(10);
  }

  return poses;
}

/*
   Random initialize particles uniformly
 */
vector<Particle> ParticleFilter::init_particles() {
  vector<Particle> particles(kParticles);

  // Create uniform distribution sampler for x, y, theta
  std::uniform_real_distribution<>
    u_x(map.min_x * map.resolution, map.max_x * map.resolution),
    u_y(map.min_y * map.resolution, map.max_y * map.resolution),
    // u_x(/*map.min_x*/ 350 * map.resolution, /*map.max_x*/ 450 * map.resolution),
    // u_y(/*map.min_y*/ 420 * map.resolution, /*map.max_y*/ 500 * map.resolution),
    u_theta(-PI(), PI());

  size_t i=0;
  while (i < kParticles) {

    Pose p(u_x(gen), u_y(gen), u_theta(gen));

    if (map.inside(p)) {
      particles[i] = p;
      ++i;
    }
  }

  return particles;
}

/*
   Perform 2-D Bresenham ray-tracing on the map.
 */
int ParticleFilter::Bresenham_ray_tracing(
    const int x_start, const int y_start,
    const int dx, const int dy,
    const Map& map) {

  // TODO
  return 0;
}

/*
   Perform 2-D naive ray-tracing on the map.
 */
int ParticleFilter::naive_ray_tracing(
    const FLOAT x0, const FLOAT y0,
    const FLOAT dx, const FLOAT dy,
    const Map& map) {

  auto x = x0, y = y0;
  constexpr FLOAT eps = 0.8; // 1e-5;
  for (size_t i=0; i<Laser::MaxRange; ++i) {
    x += dx;
    y += dy;

    size_t ix = x / map.resolution;
    size_t iy = y / map.resolution;

    // Since ix are unsigned, if x is negative, ix will be a very large number
    if (ix >= map.max_x || iy >= map.max_y)
      break;

    if (map.prob[ix][iy] > 1. - eps) {
      // cv::circle(simulation, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 255), 1);
      return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
  }

  return Laser::MaxRange - 1;
}

/*
   Perform 2-D ray-tracing using either naive approach or Bresenham
 */
int ParticleFilter::simulate_laser_beam(
    const FLOAT x, const FLOAT y,
    const FLOAT dx, const FLOAT dy,
    const Map& map, RayTracingAlgorithm rta) {

  switch (rta) {
    case RayTracingAlgorithm::Naive:
      return naive_ray_tracing(x, y, dx, dy, map);
    case RayTracingAlgorithm::Bresenham:
      // TODO
      // Don't just pass floating point and downcast it to integer
      return Bresenham_ray_tracing(x, y, dx, dy, map);
  }
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void ParticleFilter::simulate_laser_scan(Measurement& m, const Pose& pose, const Map& map) {

  // Transformation from robot pose to lidar pose
  const Pose R2L(5.66628200000001, -24.349396000000013, 0);

  for (size_t i=0; i<m.size(); ++i) {
    Pose lidar = pose + R2L;

    FLOAT x0 = lidar.x;
    FLOAT y0 = lidar.y;

    // minus 90 degree because the first beam start from the right
    FLOAT dx = std::cos(lidar.theta + float(i) / 180 * PI() - PI() / 2);
    FLOAT dy = std::sin(lidar.theta + float(i) / 180 * PI() - PI() / 2);

    m[i] = simulate_laser_beam(x0, y0, dx, dy, map);
  }
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> ParticleFilter::compute_likelihood(
    const vector<Measurement>& simulated_measurements,
    const Measurement& measurement) {

  vector<FLOAT> likelihoods(kParticles, 0);

  // Compare every particle with every model pairwisely
  for (size_t i=0; i<kParticles; ++i) {
    for (size_t j=0; j<Laser::kBeamPerScan; ++j) {
      auto l = SensorModel::eval(simulated_measurements[i][j], measurement[j]);
      likelihoods[i] += std::log(l);
    }
  }

  auto max_log = *std::max_element(likelihoods.begin(), likelihoods.end());
  for (size_t i=0; i<likelihoods.size(); ++i)
    likelihoods[i] = std::exp(likelihoods[i] - max_log);

  return likelihoods;
}

/*
   Low variance re-sampling
 */
vector<Particle> ParticleFilter::low_variance_resampling(
    const vector<Particle>& particles,
    const vector<FLOAT>& weights) {

  vector<Particle> new_particles;

  auto sum = std::accumulate(weights.begin(), weights.end(), 0.);
  FLOAT interval = sum / kParticles;
  assert(sum != 0);

  std::uniform_real_distribution<> u(0, interval * 0.99);
  FLOAT r = u(gen);

  FLOAT cumsum = 0;
  int counter = 0;

  for (size_t i=0; i<kParticles; ++i) {
    auto s = r + interval * i;

    while (!(cumsum + weights[counter] > s)) {
      cumsum += weights[counter];
      ++counter;
    }

    int idx = counter;
    assert(idx >= 0 && idx < particles.size());
    new_particles.push_back(particles[idx]);
  }

  assert(new_particles.size() == particles.size());

  return new_particles;
}

/*
   <Motion Model>
 */
void ParticleFilter::update_particles_through_motion_model(
    const Pose& delta,
    vector<Pose>& poses) {

  std::normal_distribution<>
    normal_x(delta.x, ParticleFilter::motion_sigma),
    normal_y(delta.y, ParticleFilter::motion_sigma),
    normal_theta(delta.theta, 2*PI() / 100 /*ParticleFilter::motion_sigma*/);

  for (size_t i=0; i<poses.size(); ++i) {
    poses[i].x += normal_x(gen);
    poses[i].y += normal_y(gen);
    poses[i].theta += normal_theta(gen);
  }
}

/*
   Compute particle centroid (weighted sum)
 */
Particle ParticleFilter::compute_particle_centroid(
    const std::vector<Particle>& particles,
    const std::vector<FLOAT>& weights) {

  Particle centroid;
  for (size_t i=0; i<particles.size(); ++i) {
    centroid += particles[i] * weights[i];
  }

  centroid /= particles.size();

  return centroid;
}

/*
   Show particles on map
 */
void ParticleFilter::show_particles_on_map(const vector<Particle>& particles) const {
  cv::Mat img = map.cv_img.clone();

  for (size_t i=0; i<particles.size(); ++i) {

    size_t ix = particles[i].x / map.resolution;
    size_t iy = particles[i].y / map.resolution;

    if (ix >= map.max_x || iy >= map.max_y)
      continue;

    cv::circle(img, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 255), 1);
    // printf("ix = %zu, iy = %zu\n", ix, iy);
  }

  cv::imshow("Display window", img);
  cv::waitKey(10);
}
