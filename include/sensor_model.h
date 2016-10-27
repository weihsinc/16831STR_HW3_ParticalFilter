#ifndef __SENSOR_MODEL_H_
#define __SENSOR_MODEL_H_

#include <cmath>
#include <vector>
#include <cassert>
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

  static FLOAT* likelihoods;

  static void init_lookup_table() {
    likelihoods = new FLOAT[Laser::MaxRange * Laser::MaxRange];

    printf("Initializing the lookup table for sensor model (this might take a while) ...\n");
    for (int z=0; z<Laser::MaxRange; ++z) {
      for (int x=0; x<Laser::MaxRange; ++x) {
	likelihoods[z * Laser::MaxRange + x] = std::log(_eval(x, z));
      }
    }
    printf("Done.");
  }

  static FLOAT eval(int x, int z) {
    assert (x < Laser::MaxRange && z < Laser::MaxRange);
    return likelihoods[z * Laser::MaxRange + x];
  }

  static FLOAT _eval(int x, int z) {

    FLOAT l0 = std::exp(-0.5 * pow((FLOAT) (x - z) / sigma, 2));
    FLOAT l1 = std::exp(-exp_decay * x);
    FLOAT l2 = 1. / Laser::MaxRange;
    FLOAT l3 = x > Laser::MaxRange - 50 ? 1. : 0.;

    FLOAT L = l0 * weights[0] + l1 * weights[1] + l2 * weights[2] + l3 * weights[3];
    return L;
  }
};

#endif // __SENSOR_MODEL_H_
