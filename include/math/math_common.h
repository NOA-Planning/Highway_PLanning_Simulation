#pragma once

#include <math.h>

#include <cmath>

namespace ahrs {
namespace math {
/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);
}  // namespace math
}  // namespace ahrs