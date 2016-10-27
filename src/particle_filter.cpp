#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>

#include <utils.h>
#include <particle_filter.h>
using namespace std;

bool ParticleFilter::show_ray_tracing;

static std::random_device rd;
static std::mt19937 gen(rd());

Pose MotionModel::sample() {
  return Pose(x(gen), y(gen), theta(gen));
}

ParticleFilter::ParticleFilter(
    const Map& map,
    const size_t kParticles,
    const MotionModel& motion_model,
    const size_t nThreads):
  map(map), kParticles(kParticles), threads(nThreads),
  particles(kParticles), particle_mask(kParticles), flag(false),
  pose_gnd(400 * map.resolution, 414 * map.resolution, -PI/2-PI/10-PI/6+41./180.*PI),
  motion_model(motion_model),
  simulated_measurements(kParticles, Measurement(Laser::kBeamPerScan)) {
    init_particles();
    cv_img = map.cv_img.clone();
}

/*
   Perform particle filter algorithm
   Return the estimated pose over time
 */
vector<Pose> ParticleFilter::operator () (const vector<SensorMsg*> sensor_msgs) {

  vector<Pose> poses;

  // 1) Perform particle filter algorithm iteratively
  for (size_t i=1; i<sensor_msgs.size(); ++i) {

    auto t_start_total = timer_start();

    img = cv_img.clone();
    // simulation_naive = cv_img.clone();
    simulation_bresenham = cv_img.clone();

    auto p0 = sensor_msgs[i-1]->pose;
    auto p1 = sensor_msgs[i]->pose;
    auto dt = sensor_msgs[i]->timestamp - sensor_msgs[i-1]->timestamp;

    update_one_particle_through_motion_model(pose_gnd, p0, p1, dt, true);
    /*
    Measurement m(Laser::kBeamPerScan);
    flag = true;
    simulate_laser_scan(m, pose_gnd);
    flag = false;
    */

    update_particles_through_motion_model(p0, p1, dt);

    if (sensor_msgs[i]->type() == SensorMsg::Odometry) {
      // Do nothing
    }
    else {
      Laser* laser = dynamic_cast<Laser*>(sensor_msgs[i]);

      simulate_laser_scan_all_particles();

      if (show_ray_tracing) {
	// cv::imshow("Ray-Tracing Simulation (Naive)", simulation_naive);
  	cv::imshow("Ray-Tracing Simulation (Bresenham)", simulation_bresenham);
      }

      // Compute likelihood by asking sensor model
      auto likelihoods = compute_likelihood(laser->ranges);

      /*
      for (size_t j=0; j<particles.size(); ++j)
        likelihoods[j] = std::max(0.01f, likelihoods[j]);
      // */

      // Only keep particles inside the map
      for (size_t j=0; j<particles.size(); ++j) {
	if (!map.inside(particles[j]))
	  likelihoods[j] *= 0;
      }

      /*
      // Compute the centroid of particles (mean)
      auto centroid = compute_particle_centroid(likelihoods);
      cout << "pose[" << i << "] = " << centroid << endl;
      // */

      show_particles_on_map(likelihoods);

      for (size_t j=0; j<laser->ranges.size(); ++j) {
	auto& r = laser->ranges[j];
	int x = (r * std::cos(pose_gnd.theta + float(j) / 180 * PI - PI / 2) + pose_gnd.x) / map.resolution;
	int y = (r * std::sin(pose_gnd.theta + float(j) / 180 * PI - PI / 2) + pose_gnd.y) / map.resolution;
	cv::circle(img,
	    cv::Point(x, y),
	    0, cv::Scalar(0, 0, float(255) * j / 180), 1);
      }

      // Low-Variance Resampling
      // if (i > 100)
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
    u_x(map.min_x * map.resolution, map.max_x * map.resolution),
    u_y(map.min_y * map.resolution, map.max_y * map.resolution),
    u_theta(-PI, PI);
    /*u_x(654 * map.resolution, 654 * map.resolution),
    u_y(642 * map.resolution, 642 * map.resolution),
    u_theta(-PI, PI);*/
    /*u_x(350 * map.resolution, 450 * map.resolution),
    u_y(378 * map.resolution, 478 * map.resolution),
    u_theta(-PI/2-PI/10, -PI/2-PI/10);*/

  size_t i=0;
  while (i < kParticles) {

    Pose p(u_x(gen), u_y(gen), u_theta(gen));

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
  if (flag) {
  printf("\n\n\33[33m-----------------------------------\33[0m\n");
  printf("(x0, y0) = (%d, %d), (dx, dy) = (%d, %d), axis = %d, (sx, sy) = (%d, %d)\n",
      x0, y0, dx, dy, axis, sx, sy);
  }
  // */

  dx = std::abs(dx);
  dy = std::abs(dy);

  int ax = dx << 1;
  int ay = dy << 1;

  const int L = std::max(dx, dy);
  int a = L << 1;

  int decx = ax - L;
  int decy = ay - L;

  if (flag) {
    cv::circle(img, cv::Point(x0, y0), 0, cv::Scalar(0, 0, 255), 5);
    /*
    printf("(|dx|, |dy|) = (%d, %d), (ax, ay) = (%d, %d), L = %d, a = %d, (decx, decy) = (%d, %d)\n",
      dx, dy, ax, ay, L, a, decx, decy);
      */
  }
  // */

  // Use unsigned integer so that we don't have to check x < 0 or not
  size_t x = x0;
  size_t y = y0;

  constexpr FLOAT threshold = 0.2;

  int dist = Laser::MaxRange - 1;

  for (int i=0; i<L; ++i) {

    if (flag) {
      // printf("theta = %f\n", theta);
      // cv::circle(img, cv::Point(x, y), 0, cv::Scalar(float(255) * theta, 0, 0), 1);
      // printf("i = %d, (x, y) = (%zu, %zu)\n", i, x, y);
    }
    // */

    if (x >= map.max_x || y >= map.max_y)
      break;

    if (map.prob[x * map.size_y + y] > threshold) {

      if (show_ray_tracing)
	cv::circle(simulation_bresenham, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);

      if (flag) {
	cv::circle(img, cv::Point(x, y), 0, cv::Scalar(0, float(255) * theta, 0), 1);
      }

      dist = (float) sqrt(square(int(x) - x0) + square(int(y) - y0)) * map.resolution;

      break;
    }

    if (axis == 0) x += sx; else trace_one_step(x, a);
    if (axis == 1) y += sy; else trace_one_step(y, a);
  }

  /*
  if (flag)
    printf("dist = %d\n", dist);
    */

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

  int dist = -1;
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
  assert(dist != -1);
  return dist;
}

void ParticleFilter::simulate_laser_scan_all_particles() {

  auto t_start = timer_start();

  size_t kParticlesPerThread = kParticles / threads.size();

  for (size_t i=0; i<threads.size(); ++i) {
    size_t i_begin = i*kParticlesPerThread;
    size_t i_end = i == threads.size() - 1 ? kParticles : (i+1) * kParticlesPerThread;

    threads[i] = std::thread([this, i_begin, i_end] {
	this->simulate_laser_scan_some_particles(i_begin, i_end);
    });
  }

  for (auto& t : threads)
    t.join();

  for (size_t i=0; i<kParticles; ++i) {
    int sum = 0;
    for (size_t j=0; j<Laser::kBeamPerScan; ++j)
      sum += simulated_measurements[i][j];
    /*    
    if (sum == 0) {
        printf("idx = %zu\n", i);
        cout << simulated_measurements[i] << endl;
    }*/
    particle_mask[i] = (sum == 0);
  }

  printf("Took %g to do ray-tracing\n", timer_end(t_start));
}

void ParticleFilter::simulate_laser_scan_some_particles(
    size_t i_begin, size_t i_end) {
  // printf("i_begin = %zu, i_end = %zu\n", i_begin, i_end); 

  for (size_t i=i_begin; i<i_end; ++i)
    simulate_laser_scan(simulated_measurements[i], particles[i]);
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void ParticleFilter::simulate_laser_scan(Measurement& m, const Pose& pose) {

  // Transformation from robot pose to lidar pose
  const Pose R2L(25.0*cos(pose.theta), 25.0*sin(pose.theta), 0);

  Pose lidar = pose + R2L;

  FLOAT x0 = lidar.x;
  FLOAT y0 = lidar.y;

  for (size_t i=0; i<m.size(); ++i) {
    // minus 90 degree because the first beam start from the right
    FLOAT dx = std::cos(lidar.theta + float(i) / 180 * PI - PI / 2);
    FLOAT dy = std::sin(lidar.theta + float(i) / 180 * PI - PI / 2);

    theta = float(i) / 180;
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
      likelihoods[i] += l;
    }
  }

  /*
  auto max_e = *std::max_element(likelihoods.begin(), likelihoods.end());
  auto min_e = *std::min_element(likelihoods.begin(), likelihoods.end());
  
  for (size_t i=0; i<kParticles; ++i) {
    likelihoods[i] = (likelihoods[i] - min_e) / (max_e - min_e);
  }
  // */

  auto max_log = *std::max_element(likelihoods.begin(), likelihoods.end());
  for (size_t i=0; i<likelihoods.size(); ++i)
    likelihoods[i] = std::exp(likelihoods[i] - max_log);
  // */

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
    const Pose& p0, const Pose& p1, TIME_T dt) {

  auto t_start = timer_start();

  for (auto& p : particles)
    update_one_particle_through_motion_model(p, p0, p1, dt);

  printf("Took %g to update particle through motion model\n", timer_end(t_start));
}

void ParticleFilter::update_one_particle_through_motion_model(
    Particle& p, const Pose& p0, const Pose& p1, TIME_T dt, bool deterministic) {

  auto delta = p1 - p0;

  auto c = std::cos(p.theta - p0.theta);
  auto s = std::sin(p.theta - p0.theta);

  p.x += (c * delta.x - s * delta.y);
  p.y += (s * delta.x + c * delta.y);
  p.theta += delta.theta;

  if (!deterministic)
    p += motion_model.sample() * std::sqrt(dt);
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
void ParticleFilter::show_particles_on_map(const std::vector<FLOAT>& likelihoods) {

  const cv::Scalar RED(0, 0, 255);
  const cv::Scalar GREEN(0, 255, 0);

  std::vector<size_t> indices(kParticles);
  for (size_t i=0; i<kParticles; ++i)
    indices[i] = i;

  sort(indices.begin(), indices.end(), 
    [&likelihoods] (const size_t &i, const size_t& j) -> bool {
      return likelihoods[i] < likelihoods[j];
  });

  // Plot GREEN (sum != 0) particles
  for (size_t i=0; i<particles.size(); ++i) {
    size_t j = indices[i];

    size_t ix = particles[j].x / map.resolution;
    size_t iy = particles[j].y / map.resolution;

    if (ix >= map.max_x || iy >= map.max_y)
      continue;

    if (particle_mask[i])
      continue;
    
    // cout << likelihoods[j] << endl;    
    int darkness = 255 * (1 - likelihoods[j]);
    cv::circle(img, cv::Point(ix, iy), 1, cv::Scalar(0, darkness, 255), 2 + likelihoods[j]);
  }

  // Plot RED (sum == 0) particles
  for (size_t i=0; i<particles.size(); ++i) {

    size_t ix = particles[i].x / map.resolution;
    size_t iy = particles[i].y / map.resolution;

    if (ix >= map.max_x || iy >= map.max_y)
      continue;

    if (!particle_mask[i])
      continue;

    // printf("ix = %zu, iy = %zu\n", ix, iy);
    cv::circle(img, cv::Point(ix, iy), 0, GREEN, 2);
  }

  cv::circle(cv_img, map.to_idx(pose_gnd), 0, cv::Scalar(128, 0, 255), 1);
  cv::circle(img, map.to_idx(pose_gnd), 0, cv::Scalar(255, 0, 0), 1);
  cv::circle(img, map.to_idx(pose_gnd), 15, cv::Scalar(255, 0, 0), 2);

  cv::imshow("Display window", img);
  cv::waitKey(10);
}
