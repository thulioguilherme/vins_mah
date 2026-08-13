#ifndef VINS_MAH_VIO_TYPES_H_
#define VINS_MAH_VIO_TYPES_H_

namespace vins_mah
{

enum class ExtrinsicEstimationMode {
  FIXED = 0,
  OPTIMIZE = 1,
  CALIBRATE = 2
};

enum class EstimatorState {
  INITIALIZATION,
  OPTIMIZATION
};


enum class MarginalizationStrategy {
  REMOVE_OLDEST,
  REMOVE_SECOND_NEWEST
};

namespace StateSize {
  constexpr int POSE = 7;
  constexpr int SPEED_BIAS = 9;
  constexpr int FEATURE = 1;
}

namespace StateOrder {
  constexpr int POSITION = 0;
  constexpr int ROTATION = 3;
  constexpr int VELOCITY = 6;
  constexpr int ACCELERATION_BIAS = 9;
  constexpr int GYROSCOPE_BIAS = 12;
}

namespace NoiseOrder {
  constexpr int ACCELERATION_NOISE = 0;
  constexpr int GYROSCOPE_NOISE = 3;
  constexpr int ACCELERATION_RANDOM_WALK = 6;
  constexpr int GYROSCOPE_RANDOM_WALK = 9;
}

} // namespace vins_mah

#endif // VINS_MAH_VIO_TYPES_H_
