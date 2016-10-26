#ifndef __SENSOR_MODEL_H_
#define __SENSOR_MODEL_H_

#include <cmath>
#include <vector>
#include <sensor_msg.h>

/*
   <Sensor Model>
   1. Measurement Noise    - Gaussian distribution centered at measurement z
   2. Unexpected Obstacles - Exponential decay
   3. Random Measurement   - Uniform distribution
   4. Max range            - a peak at z_max
 */

class SensorModel {
public:
  static std::vector<FLOAT> weights;
  static FLOAT exp_decay;
  static FLOAT sigma;

  static FLOAT eval(size_t x, size_t z) {
    
    const FLOAT denom = 1.; // 1. / (sqrt(2 * PI()) * ParticleFilter::sigma);

    FLOAT l0 = denom * std::exp(-0.5 * pow((FLOAT) (x - z) / sigma, 2));
    // FLOAT l1 = x < z ? (std::exp(-exp_decay * x) - std::exp(-exp_decay * z)) : 0.;
    FLOAT l1 = std::exp(-exp_decay * x);
    FLOAT l2 = 1. / Laser::MaxRange;
    FLOAT l3 = x > Laser::MaxRange - 50 ? 1. : 0.;

    FLOAT L = l0 * weights[0] + l1 * weights[1] + l2 * weights[2] + l3 * weights[3];
    return L;
  }
};

#endif // __SENSOR_MODEL_H_
