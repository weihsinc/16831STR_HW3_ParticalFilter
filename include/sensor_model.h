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
  static std::vector<FLOAT> sensor_model_weights;
  static FLOAT exp_decay;
  static FLOAT sigma;

  static float eval(int x, int z) {
    
    static std::vector<FLOAT> likelihoods(4);
    const FLOAT denom = 1.; // 1. / (sqrt(2 * PI()) * ParticleFilter::sigma);

    FLOAT L = 0;

    likelihoods[0] = denom * std::exp(-0.5 * pow((FLOAT) (x - z) / sigma, 2));
    likelihoods[1] = x < z ? std::exp(-exp_decay * x) : 0.;
    likelihoods[2] = 1. / Laser::MaxRange;
    likelihoods[3] = x > Laser::MaxRange - 50 ? 1. : 0.;

    for (size_t j=0; j<4; ++j)
      L += likelihoods[j] * sensor_model_weights[j];

    return L;
  }
};

#endif // __SENSOR_MODEL_H_
