#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>

#include <utils.h>
#include <particle_filter.h>
using namespace std;

std::vector<FLOAT> ParticleFilter::sensor_model_weights(4);
FLOAT ParticleFilter::exp_decay;
FLOAT ParticleFilter::sigma;
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

      compute_sensor_model(models, laser->ranges);
      
      for (size_t j=0; j<kParticles; ++j)
	simulate_laser_scan(simulated_measurements[j], particles[j], map);

      auto likelihoods = compute_likelihood(simulated_measurements, models);
    }
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
    u_x(map.min_x, map.max_x),
    u_y(map.min_y, map.max_y),
    u_theta(-PI(), PI());

  for (size_t i=0; i<kParticles; ++i)
    particles[i] = Pose(u_x(gen), u_y(gen), u_theta(gen));

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
  constexpr FLOAT eps = 1e-5;
  for (size_t i=0; i<Laser::MaxRange; ++i) {
    x += dx;
    y += dy;

    size_t ix = x;
    size_t iy = y;

    // Since ix are unsigned, if x is negative, ix will be a very large number
    if (ix > map.max_x || iy > map.max_y)
      break;

    /*
    printf("(x, y) = (%f, %f), (ix, iy) = (%zu, %zu), prob = %f\n",
	x, y, ix, iy, map.prob[ix][iy]);
	*/

    if (map.prob[ix][iy] > 1. - eps)
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
    FLOAT dx = std::cos(lidar.theta - PI() / 2);
    FLOAT dy = std::sin(lidar.theta - PI() / 2);

    m[i] = simulate_laser_beam(x0, y0, dx, dy, map);
  }
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
void ParticleFilter::compute_sensor_model_per_beam(PDF& pdf, int z) {

  // Precompute the denominator in Gaussian distribution
  // denom = \frac{1}{\sqrt {2\pi} \sigma}
  const FLOAT denom = 1. / (sqrt(2 * PI()) * ParticleFilter::sigma);
  auto& weights = ParticleFilter::sensor_model_weights;

  vector<FLOAT> likelihoods(4);
  FLOAT sum = 0;

  std::fill(pdf.begin(), pdf.end(), 0);
  for (size_t i=0; i<pdf.size(); ++i) {

    likelihoods[0] = denom * std::exp(0.5 * pow((FLOAT) (i - z) / ParticleFilter::sigma, 2));
    likelihoods[1] = std::exp(-ParticleFilter::exp_decay * i);
    likelihoods[2] = 1. / Laser::MaxRange;
    likelihoods[3] = i == Laser::MaxRange ? 1. : 0.;

    for (size_t j=0; j<4; ++j)
      pdf[i] += likelihoods[j] * weights[j];

    sum += pdf[i];
  }

  for (size_t i=0; i<pdf.size(); ++i)
    pdf[i] /= sum;
}

/* 
   <Sensor PDF>
   Turn a laser beam (180 integers) into a vector of probability distributions
 */
void ParticleFilter::compute_sensor_model(vector<PDF>& models, const vector<int> &z) {
  for (size_t i=0; i<z.size(); ++i)
    compute_sensor_model_per_beam(models[i], z[i]);
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> ParticleFilter::compute_likelihood(
    const vector<Measurement>& simulated_measurements,
    const vector<PDF>& models) {

  vector<FLOAT> likelihoods(kParticles, 0);

  // Compare every particle with every model pairwisely
  for (size_t i=0; i<kParticles; ++i) {
    for (size_t j=0; j<Laser::kBeamPerScan; ++j) {
      auto m = simulated_measurements[i][j];
      assert(m < models[j].size());
      likelihoods[i] += models[j][m];
    }
  }

  return likelihoods;
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
    normal_theta(delta.theta, ParticleFilter::motion_sigma);

  for (size_t i=0; i<poses.size(); ++i) {
    poses[i].x += normal_x(gen);
    poses[i].y += normal_y(gen);
    poses[i].theta += normal_theta(gen);
  }
}
