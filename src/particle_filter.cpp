#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>

#include <utils.h>
#include <particle_filter.h>
using namespace std;

FLOAT ParticleFilter::motion_sigma;
bool ParticleFilter::show_ray_tracing;

std::random_device ParticleFilter::rd;
std::mt19937 ParticleFilter::gen(rd());

ParticleFilter::ParticleFilter(
    const Map& map,
    const size_t kParticles,
    const size_t nThreads):
  map(map), kParticles(kParticles), threads(nThreads),
  particles(kParticles),
  simulated_measurements(kParticles, Measurement(Laser::kBeamPerScan)) {
    init_particles();
}

/*
   Perform particle filter algorithm
   Return the estimated pose over time
 */
vector<Pose> ParticleFilter::operator () (const vector<SensorMsg*> sensor_msgs) {

  vector<Pose> poses;

  // 1) Perform particle filter algorithm iteratively
  for (size_t i=1; i<sensor_msgs.size(); ++i) {

    show_particles_on_map();

    auto t_start_total = timer_start();

    auto p0 = sensor_msgs[i-1]->pose;
    auto p1 = sensor_msgs[i]->pose;

    update_particles_through_motion_model(p0, p1);

    if (sensor_msgs[i]->type() == SensorMsg::Odometry) {
      // Do nothing
    }
    else {
      Laser* laser = dynamic_cast<Laser*>(sensor_msgs[i]);

      // Ray tracing
      if (show_ray_tracing) {
	simulation_naive = map.cv_img.clone();
	simulation_bresenham = map.cv_img.clone();
      }

      simulate_laser_scan_all_particles();

      if (show_ray_tracing) {
	cv::imshow("Ray-Tracing Simulation (Naive)", simulation_naive);
  	cv::imshow("Ray-Tracing Simulation (Bresenham)", simulation_bresenham);
      }

      // Compute likelihood by asking sensor model
      auto likelihoods = compute_likelihood(laser->ranges);

      // Only keep particles inside the map
      for (size_t j=0; j<particles.size(); ++j) {
	if (!map.inside(particles[j]))
	  likelihoods[j] *= 0;
      }

      // Compute the centroid of particles (mean)
      /*
      auto centroid = compute_particle_centroid(likelihoods);
      cout << "pose[" << i << "] = " << centroid << endl;
      */

      // Low-Variance Resampling
      particles = low_variance_resampling(likelihoods);

      printf("Took \33[33m%g\33[0m in total\n\n", timer_end(t_start_total));
    }

    cv::waitKey(5);
  }

  return poses;
}

/*
   Random initialize particles uniformly
 */
void ParticleFilter::init_particles() {
  // Create uniform distribution sampler for x, y, theta
  std::uniform_real_distribution<>
    /*u_x(map.min_x * map.resolution, map.max_x * map.resolution),
    u_y(map.min_y * map.resolution, map.max_y * map.resolution),*/
    u_x(350 * map.resolution, 450 * map.resolution),
    u_y(420 * map.resolution, 500 * map.resolution),
    u_theta(-PI, PI);

  size_t i=0;
  while (i < kParticles) {

    Pose p(u_x(gen), u_y(gen), PI/2 /*u_theta(gen)*/);

    if (map.inside(p)) {
      particles[i] = p;
      ++i;
    }
  }
}

// Use C macro ##, which is a token-pasting operator
#define trace_one_step(x, a) { \
  if (dec##x >= 0) { \
    dec##x -= a; \
    x += s##x; \
  } \
  dec##x += a##x; \
}

#define square(x) ((x) * (x))

// sign of x, return either -1, 0, or 1
inline int sign(int x) { return (x > 0) ? 1 : ((x < 0) ? -1 : 0); }

/*
   Perform 2-D Bresenham ray-tracing on the map.
 */
int ParticleFilter::Bresenham_ray_tracing(
    const int x0, const int y0,
    int dx, int dy, bool debug) {

  // x-axis is 0-th axis, y-axis is 1-th axis
  // If dx > dy, then (principal) axis is 0. Otherwise (principal) axis is 1.
  int axis = std::abs(dx) < std::abs(dy);

  int sx = sign(dx);
  int sy = sign(dy);

  /*
  printf("\n\n\33[33m-----------------------------------\33[0m\n");
  printf("(x0, y0) = (%d, %d), (dx, dy) = (%d, %d), axis = %d, (sx, sy) = (%d, %d)\n",
      x0, y0, dx, dy, axis, sx, sy);
  // */

  dx = std::abs(dx);
  dy = std::abs(dy);

  int ax = dx << 1;
  int ay = dy << 1;

  const int L = std::max(dx, dy);
  int a = L << 1;

  int decx = ax - L;
  int decy = ay - L;

  /*
  printf("(|dx|, |dy|) = (%d, %d), (ax, ay) = (%d, %d), L = %d, a = %d, (decx, decy) = (%d, %d)\n",
      dx, dy, ax, ay, L, a, decx, decy);
  // */

  // Use unsigned integer so that we don't have to check x < 0 or not
  size_t x = x0;
  size_t y = y0;

  constexpr FLOAT threshold = 0.2;

  int dist = Laser::MaxRange - 1;

  for (int i=0; i<L; ++i) {
    /*
    printf("i = %d, (x, y) = (%zu, %zu)\n", i, x, y);
    // */

    if (x >= map.max_x || y >= map.max_y)
      break;

    if (map.prob[x * map.size_y + y] > threshold) {

      if (show_ray_tracing)
	cv::circle(simulation_bresenham, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);

      dist = (float) sqrt(square(int(x) - x0) + square(int(y) - y0)) / map.resolution;
      break;
    }

    if (axis == 0) x += sx; else trace_one_step(x, a);
    if (axis == 1) y += sy; else trace_one_step(y, a);
  }

  return dist;
}

/*
   Perform 2-D naive ray-tracing on the map.
 */
int ParticleFilter::naive_ray_tracing(
    const FLOAT x0, const FLOAT y0,
    const FLOAT dx, const FLOAT dy) {

  auto x = x0, y = y0;
  constexpr FLOAT threshold = 0.2;
  for (size_t i=0; i<Laser::MaxRange; ++i) {
    x += dx;
    y += dy;

    size_t ix = x / map.resolution;
    size_t iy = y / map.resolution;

    // Since ix are unsigned, if x is negative, ix will be a very large number
    if (ix >= map.max_x || iy >= map.max_y)
      break;

    if (map.prob[ix * map.size_y + iy] > threshold) {
      if (show_ray_tracing)
	cv::circle(simulation_naive, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 255), 1);

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
    RayTracingAlgorithm rta) {

  int dist = 0;
  switch (rta) {
    case RayTracingAlgorithm::Naive:
      dist = naive_ray_tracing(x, y, dx, dy);
      break;
    case RayTracingAlgorithm::Bresenham:

      int x0 = x / map.resolution;
      int y0 = y / map.resolution;
      int delta_x = (dx * Laser::MaxRange) / map.resolution;
      int delta_y = (dy * Laser::MaxRange) / map.resolution;

      // printf("(x0, y0) = (%d, %d), delta = (%d, %d)\n", x0, y0, delta_x, delta_y);
      dist = Bresenham_ray_tracing(x0, y0, delta_x, delta_y);
      break;
  }
  return dist;
}

void ParticleFilter::simulate_laser_scan_all_particles() {

  auto t_start = timer_start();

  size_t kParticlesPerThread = kParticles / threads.size();

  for (size_t i=0; i<threads.size(); ++i) {
    size_t i_begin = i*kParticles;
    size_t i_end = std::min((i+1) * kParticlesPerThread, kParticles);

    threads[i] = std::thread([this, i_begin, i_end] {
	this->simulate_laser_scan_some_particles(i_begin, i_end);
    });
  }

  for (auto& t : threads)
    t.join();

  // this->simulate_laser_scan_some_particles(0, kParticles);

  printf("Took %g to do ray-tracing\n", timer_end(t_start));
}

void ParticleFilter::simulate_laser_scan_some_particles(
    size_t i_begin, size_t i_end) {

  for (size_t i=i_begin; i<i_end; ++i)
    simulate_laser_scan(simulated_measurements[i], particles[i]);
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void ParticleFilter::simulate_laser_scan(Measurement& m, const Pose& pose) {

  // Transformation from robot pose to lidar pose
  const Pose R2L(5.66628200000001, -24.349396000000013, 0);

  for (size_t i=0; i<m.size(); ++i) {
    Pose lidar = pose + R2L;

    FLOAT x0 = lidar.x;
    FLOAT y0 = lidar.y;

    // minus 90 degree because the first beam start from the right
    FLOAT dx = std::cos(lidar.theta + float(i) / 180 * PI - PI / 2);
    FLOAT dy = std::sin(lidar.theta + float(i) / 180 * PI - PI / 2);

    m[i] = simulate_laser_beam(x0, y0, dx, dy, RayTracingAlgorithm::Bresenham);
  }
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> ParticleFilter::compute_likelihood(
    const Measurement& measurement) {

  auto t_start = timer_start();
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

  printf("Took %g to compute likelihood\n", timer_end(t_start));

  return likelihoods;
}

/*
   Low variance re-sampling
 */
vector<Particle> ParticleFilter::low_variance_resampling(
    const vector<FLOAT>& weights) {

  vector<Particle> new_particles;

  FLOAT sum = 0;
  for (auto& w : weights)
    sum += w;

  FLOAT interval = sum / kParticles;
  assert(sum != 0);
  assert(sum == sum);

  std::uniform_real_distribution<> u(0, interval * 0.99);
  FLOAT r = u(gen);

  FLOAT cumsum = 0;
  size_t counter = 0;

  for (size_t i=0; i<kParticles; ++i) {
    auto s = r + interval * i;

    // printf("cumsum = %f, weights[%zu] = %f, s = %f\n", cumsum, counter, weights[counter], s);
    while (!(cumsum + weights[counter] > s)) {
      cumsum += weights[counter];
      ++counter;
    }

    size_t idx = counter;
    if (idx >= particles.size()) {
      printf("\33[31m[Error]\33[0m\n");
      printf("idx = %zu, counter = %zu, sum = %f, cumsum = %f, i = %zu, r = %f, interval = %f, weights[%zu] = %f\n",
	  idx, counter, sum, cumsum, i, r, interval, counter, weights[counter]);

      exit(-1);
    }
    new_particles.push_back(particles[idx]);
  }

  assert(new_particles.size() == particles.size());

  return new_particles;
}

/*
   <Motion Model>
 */
void ParticleFilter::update_particles_through_motion_model(
    const Pose& p0, const Pose& p1) {

  auto t_start = timer_start();

  auto delta = p1 - p0;

  std::normal_distribution<>
    normal_x(0, ParticleFilter::motion_sigma),
    normal_y(0, ParticleFilter::motion_sigma),
    normal_theta(0, 2*PI / 100);

  for (auto& p : particles) {
    auto c = std::cos(p.theta - p0.theta);
    auto s = std::sin(p.theta - p0.theta);

    p.x += (c * delta.x - s * delta.y) + normal_x(gen);
    p.y += (s * delta.x + c * delta.y) + normal_y(gen);
    p.theta += delta.theta + normal_theta(gen);
  }

  printf("Took %g to update particle through motion model\n", timer_end(t_start));
}

/*
   Compute particle centroid (weighted sum)
 */
Particle ParticleFilter::compute_particle_centroid(
    const std::vector<FLOAT>& weights) const {

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
void ParticleFilter::show_particles_on_map() const {
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
