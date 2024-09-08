// =============================================================================
// Authors: Tora
// =============================================================================

#ifndef TORA_CONSTANT_DEFINES_H_
#define TORA_CONSTANT_DEFINES_H_

namespace Tora {

using Real = double;

static const Real PI = 3.141592653589793238462643383279;
static const Real PI_2 = 1.570796326794896619231321691639;
static const Real PI_4 = 0.785398163397448309615660845819;
static const Real RAD_TO_DEG = 180.0 / 3.1415926535897932384626433832795;
static const Real DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;
static const Real RPM_TO_RADS = 2.0 * 3.1415926535897932384626433832795 / 60.0;
static const Real RADS_TO_RPM = 60.0 / 2.0 * 3.1415926535897932384626433832795;

static const Real SQRT_2 = 1.41421356237309504880;
static const Real SQRT_1_2 = 0.70710678118654752440;

static const Real E = 2.71828182845904523536;
static const Real LOG2E = 1.44269504088896340736;
static const Real LOG10E = 0.43429448190325182765;
static const Real LN2 = 0.69314718055994530941;
static const Real LN10 = 2.30258509299404568402;

/// Clamp and modify the specified value to lie within the given limits.
template <typename T>
void ClampValue(T& value, T limitMin, T limitMax) {
  if (value < limitMin)
    value = limitMin;
  else if (value > limitMax)
    value = limitMax;
}

/// Clamp the specified value to lie within the given limits.
template <typename T>
T Clamp(T value, T limitMin, T limitMax) {
  if (value < limitMin) return limitMin;
  if (value > limitMax) return limitMax;

  return value;
}

template <typename T>
T Atan2(T mcos, T msin) {
  T ret;
  if (fabs(mcos) < 0.707) {
    ret = acos(mcos);
    if (msin < 0.0) ret = -ret;
  } else {
    ret = asin(msin);
    if (mcos < 0.0) ret = PI - ret;
  }
  return ret;
}

}  // end namespace Tora

#endif