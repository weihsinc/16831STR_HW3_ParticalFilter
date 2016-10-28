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
  particles(kParticles), motion_model(motion_model),
  simulated_measurements(kParticles, Measurement(Laser::kBeamPerScan)) {
    init_particles();
}

/*
   Perform particle filter algorithm
   Return the estimated pose over time
 */
void ParticleFilter::operator () (const vector<SensorMsg*> sensor_msgs) {

  vector<FLOAT> likelihoods(kParticles, 0);
  
  /* number of iterations for activating weight sampling. Accumulate every time lidar sensor update */
  const int resampleing_rate = 2;  
  size_t resampleing_counter = 0; 

  /* number of iterations freezed before first weight sampling. Accumulate every time odon/lidar sensor update */
  const int num_iters_before_1st_resampling = 10; 

  /* Open Vidoe Writer */
  int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
  int fps = 30;
  cv::VideoWriter outputVideo("videos/test.avi", fourcc, fps, cv::Size(800, 480));

  // 1) Perform particle filter algorithm iteratively
  for (size_t i=1; i<sensor_msgs.size(); ++i) {

    auto t_start_total = timer_start();

    img = map.cv_img.clone();
    simulation_bresenham = map.cv_img.clone();

    auto p0 = sensor_msgs[i-1]->pose;
    auto p1 = sensor_msgs[i]->pose;
    auto dt = sensor_msgs[i]->timestamp - sensor_msgs[i-1]->timestamp;

    update_particles_through_motion_model(p0, p1, dt);

    if (sensor_msgs[i]->type() == SensorMsg::Odometry) {
      // Do nothing
    }
    else {
      Laser* laser = dynamic_cast<Laser*>(sensor_msgs[i]);

      simulate_laser_scan_all_particles();

      if (show_ray_tracing)
        cv::imshow("Ray-Tracing Simulation (Bresenham)", simulation_bresenham);

      // Compute likelihood by asking sensor model
      likelihoods += compute_likelihood(laser->ranges);

      // playaround_with_GUI(laser, to_probabilities(likelihoods));

      show_particles_on_map(to_probabilities(likelihoods));
      cv::imshow("Display window", img);
      outputVideo << flip_and_crop();
      // cv::waitKey(0);

      // Only keep particles inside the map
      for (size_t j=0; j<particles.size(); ++j) {
        if (!map.inside(particles[j]))
          likelihoods[j] *= 0;
      }
      
      // Condition where we conduct low varaince sampling, every time we run
      // low varaince sampling, we reset the likelihoods to zeros.
      if (resampleing_counter % resampleing_rate == 0 
        && (i > num_iters_before_1st_resampling)) {
	    particles = low_variance_resampling(to_probabilities(likelihoods));
	    std::fill(likelihoods.begin(), likelihoods.end(), 0);
        }

      resampleing_counter++;
      printf("Took \33[33m%g\33[0m in total\n", timer_end(t_start_total));
    }

    cv::waitKey(5);
  }

  cv::waitKey(0);
}

cv::Mat ParticleFilter::flip_and_crop() const {
  cv::Mat flipped_img;
  cv::flip(img, flipped_img, 0);
  return flipped_img(cv::Rect(0, 50, 800, 480));
}

void ParticleFilter::playaround_with_GUI(Laser* laser, const std::vector<FLOAT>& likelihoods) {

  cv::namedWindow("Simulated measurement", cv::WINDOW_AUTOSIZE);
  auto max_idx = std::max_element(likelihoods.begin(), likelihoods.end()) - likelihoods.begin();
  auto best_pose = particles[max_idx];

  int keycode = -1;
  auto temp_img = img.clone();
  cv::Mat score_img;

  int counter = 0;
  while ((keycode = cv::waitKey(0)) != 27) {
    ++counter;
    img = temp_img.clone();
    show_particles_on_map(likelihoods);

    switch (keycode) {
      case 106: best_pose.theta += PI / 180; break;
      case 107: best_pose.theta -= PI / 180; break;
      case 97:  best_pose.x -= 10; break;
      case 100: best_pose.x += 10; break;
      case 119: best_pose.y -= 10; break;
      case 115: best_pose.y += 10; break;
      case 53:  SensorModel::weights[0] += 0.05; break;
      case 116: SensorModel::weights[0] -= 0.05; break;
      case 54:  SensorModel::weights[1] += 0.05; break;
      case 121: SensorModel::weights[1] -= 0.05; break;
      case 55:  SensorModel::weights[2] += 0.05; break;
      case 117: SensorModel::weights[2] -= 0.05; break;
      case 56:  SensorModel::weights[3] += 0.05; break;
      case 105: SensorModel::weights[3] -= 0.05; break;
      case 32:
	string new_wnd_name = "Simulated measurement" + to_string(counter);
	cv::namedWindow(new_wnd_name, cv::WINDOW_AUTOSIZE);
	cv::imshow(new_wnd_name, score_img);
	break;
    }

    cout << keycode << endl;

    const Pose R2L(25.0*cos(best_pose.theta), 25.0*sin(best_pose.theta), 0);
    auto lidar_pose = best_pose + R2L;

    simulate_laser_scan(simulated_measurements[max_idx], best_pose);

    score_img = cv::Mat(2*200, Laser::kBeamPerScan * 2, CV_8UC3, cv::Scalar(255, 255, 255));
    vector<float> scores;
    cout << "l0 (from gaussian) = " << endl;
    for (size_t j=0; j<laser->ranges.size(); ++j) {
      auto& r = laser->ranges[j];
      auto& r2 = simulated_measurements[max_idx][j];
      scores.push_back(SensorModel::eval(r2, r));
      cv::line(score_img, cv::Point(j*2, 2 * 200), cv::Point(j*2, (2 - scores[j]) * 200.), cv::Scalar(255, 128, 0), 2);
    }
    cv::imshow("Simulated measurement", score_img);

    cout << SensorModel::weights << endl;

    auto min_score = *std::min_element(scores.begin(), scores.end());
    auto max_score = *std::max_element(scores.begin(), scores.end());

    for (size_t j=0; j<laser->ranges.size(); ++j) {
      auto& r = laser->ranges[j];
      auto& r2 = simulated_measurements[max_idx][j];
      auto score = scores[j];
      int x = (r * std::cos(lidar_pose.theta + float(j) / 180 * PI - PI / 2) + lidar_pose.x) / map.resolution;
      int y = (r * std::sin(lidar_pose.theta + float(j) / 180 * PI - PI / 2) + lidar_pose.y) / map.resolution;

      cv::circle(img,
	  cv::Point(x, y),
	  0, cv::Scalar(0, 0, float(255) * score), 1);
    }

    cv::imshow("Display window", img);
    cv::waitKey(10);
  }
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

  dx = std::abs(dx);
  dy = std::abs(dy);

  int ax = dx << 1;
  int ay = dy << 1;

  const int L = std::max(dx, dy);
  int a = L << 1;

  int decx = ax - L;
  int decy = ay - L;

  // Use unsigned integer so that we don't have to check x < 0 or not
  size_t x = x0;
  size_t y = y0;

  constexpr FLOAT threshold = 0.2;

  int dist = Laser::MaxRange - 1;

  for (int i=0; i<L; ++i) {

    if (x >= map.max_x || y >= map.max_y)
      break;

    if (map.prob[x * map.size_y + y] > threshold) {

      if (show_ray_tracing)
        cv::circle(simulation_bresenham, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 1);

      dist = (float) sqrt(square(int(x) - x0) + square(int(y) - y0)) * map.resolution;

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

    if (map.prob[ix * map.size_y + iy] > threshold)
      return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
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

  // printf("Took %g to do ray-tracing\n", timer_end(t_start));
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
  const Pose R2L(25.0*cos(pose.theta), 25.0*sin(pose.theta), 0);

  Pose lidar = pose + R2L;

  FLOAT x0 = lidar.x;
  FLOAT y0 = lidar.y;

  // minus 90 degree because the first beam start from the right
  for (size_t i=0; i<m.size(); ++i) {
    auto theta = lidar.theta + (float(i) / 180 * PI) - PI / 2;
    FLOAT dx = std::cos(theta);
    FLOAT dy = std::sin(theta);
    m[i] = simulate_laser_beam(x0, y0, dx, dy, RayTracingAlgorithm::Bresenham);
  }
}

vector<float> ParticleFilter::to_probabilities(const vector<float>& likelihoods) {
  vector<float> prob(likelihoods.size());
  auto max_log = *std::max_element(likelihoods.begin(), likelihoods.end());
  for (size_t i=0; i<likelihoods.size(); ++i)
    prob[i] = pow(std::exp(likelihoods[i] - max_log), 0.02);
  return prob;
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> ParticleFilter::compute_likelihood(
    const Measurement& measurement) {

  vector<FLOAT> likelihoods(kParticles, 0);

  // Compare every particle with every model pairwisely
  for (size_t i=0; i<kParticles; ++i)
    for (size_t j=0; j<Laser::kBeamPerScan; ++j)
      likelihoods[i] += SensorModel::eval(simulated_measurements[i][j], measurement[j]);
  
  return likelihoods;
}

/*
   Low variance re-sampling
 */
vector<Particle> ParticleFilter::low_variance_resampling(
    const vector<FLOAT>& weights) {

  vector<Particle> new_particles;

  // Sum the weights, check whether it's nonzero and not NaN
  auto sum = arrsum(weights);
  assert(sum != 0 && sum == sum);

  FLOAT interval = sum / kParticles;

  std::uniform_real_distribution<> u(0, interval * 0.99);
  FLOAT r = u(gen);

  FLOAT cumsum = 0;
  size_t counter = 0;

  for (size_t i=0; i<kParticles; ++i) {
    FLOAT s = r + interval * i;

    while (!(cumsum + weights[counter] > s)) {
      cumsum += weights[counter];
      ++counter;
    }

    if (counter >= particles.size())
      break;

    new_particles.push_back(particles[counter]);
  }

  assert(new_particles.size() == particles.size());

  return new_particles;
}

/*
   <Motion Model>
 */
void ParticleFilter::update_particles_through_motion_model(
    const Pose& p0, const Pose& p1, TIME_T dt) {

  // auto t_start = timer_start();

  for (auto& p : particles)
    update_one_particle_through_motion_model(p, p0, p1, dt);

  // printf("Took %g to update particle through motion model\n", timer_end(t_start));
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

  auto indices = sort(likelihoods);

  // Plot particles with color from YELLOW to RED
  for (size_t i=0; i<particles.size(); ++i) {
    size_t j = indices[i]; // in the order of sorted likelihood

    size_t ix = particles[j].x / map.resolution;
    size_t iy = particles[j].y / map.resolution;

    if (ix >= map.max_x || iy >= map.max_y)
      continue;

    // Lower likelihood (yellow) => higher likelihood (red)
    auto p = likelihoods[j];
    cv::circle(img, cv::Point(ix, iy), 0, cv::Scalar(0, 255 * (1 - p), 255), 1);
  }
}
