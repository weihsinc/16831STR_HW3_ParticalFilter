#include <cmath>
#include <algorithm>
#include <random>

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
   Perform 2-D Bresenham ray-tracing on the map. Collect all the probabilities
   along the ray and return as an vector
 */
float ParticleFilter::simulate_laser_per_beam(
    const int x_start, const int y_start,
    const int dx, const int dy,
    const Map& map) {

  // TODO
  return 0;
}

/*
   Perform 2-D ray-tracing for all 180 beams in a single laser scan
 */
void ParticleFilter::simulate_laser_scan(Measurement& m, const Pose& pose, const Map& map) {

  // TODO
  for (size_t i=0; i<m.size(); ++i) {
    m[i] = simulate_laser_per_beam(0, 0, 0, 0, map);
  }

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
PDF ParticleFilter::sensor_model_per_beam(int z) {
  PDF pdf(Laser::MaxRange, 0.);
  // TODO

  // Precompute the denominator in Gaussian distribution
  // denom = \frac{1}{\sqrt {2\pi} \sigma}
  const FLOAT denom = 1. / (sqrt(2 * PI()) * ParticleFilter::sigma);
  auto& weights = ParticleFilter::sensor_model_weights;

  vector<FLOAT> likelihoods(4);
  FLOAT sum = 0;

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

  return pdf;
}

/* 
   <Sensor PDF>
   Turn a laser beam (180 integers) into a vector of probability distributions
 */
vector<PDF> ParticleFilter::sensor_model(const vector<int> &z) {

  vector<PDF> models(z.size());

  for (size_t i=0; i<z.size(); ++i)
    models[i] = sensor_model_per_beam(z[i]);

  return models;
}

/*
   Compare Laser measurements (per scan, meaning 180 feature vectors) to the
   measurements simulated at all particle's pose.
   */
vector<float> ParticleFilter::compute_likelihood(
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
