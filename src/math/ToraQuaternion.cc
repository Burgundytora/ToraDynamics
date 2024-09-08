#include "ToraQuaternion.h"

namespace Tora {

// -----------------------------------------------------------------------------
// QUATERNION OPERATIONS

double Qlength(const ToraQuaternion<double>& q) {
  return (sqrt(pow(q.e0(), 2) + pow(q.e1(), 2) + pow(q.e2(), 2) + pow(q.e3(), 2)));
}

ToraQuaternion<double> Qscale(const ToraQuaternion<double>& q, double fact) {
  ToraQuaternion<double> result;
  result.e0() = q.e0() * fact;
  result.e1() = q.e1() * fact;
  result.e2() = q.e2() * fact;
  result.e3() = q.e3() * fact;
  return result;
}

ToraQuaternion<double> Qadd(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb) {
  ToraQuaternion<double> result;
  result.e0() = qa.e0() + qb.e0();
  result.e1() = qa.e1() + qb.e1();
  result.e2() = qa.e2() + qb.e2();
  result.e3() = qa.e3() + qb.e3();
  return result;
}

ToraQuaternion<double> Qsub(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb) {
  ToraQuaternion<double> result;
  result.e0() = qa.e0() - qb.e0();
  result.e1() = qa.e1() - qb.e1();
  result.e2() = qa.e2() - qb.e2();
  result.e3() = qa.e3() - qb.e3();
  return result;
}

// Return the norm two of the quaternion. Euler's parameters have norm = 1
ToraQuaternion<double> Qnorm(const ToraQuaternion<double>& q) {
  double invlength;
  invlength = 1 / (Qlength(q));
  return Qscale(q, invlength);
}

// Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
ToraQuaternion<double> Qconjugate(const ToraQuaternion<double>& q) {
  ToraQuaternion<double> res;
  res.e0() = q.e0();
  res.e1() = -q.e1();
  res.e2() = -q.e2();
  res.e3() = -q.e3();
  return (res);
}

// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
ToraQuaternion<double> Qcross(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb) {
  ToraQuaternion<double> res;
  res.e0() = qa.e0() * qb.e0() - qa.e1() * qb.e1() - qa.e2() * qb.e2() - qa.e3() * qb.e3();
  res.e1() = qa.e0() * qb.e1() + qa.e1() * qb.e0() - qa.e3() * qb.e2() + qa.e2() * qb.e3();
  res.e2() = qa.e0() * qb.e2() + qa.e2() * qb.e0() + qa.e3() * qb.e1() - qa.e1() * qb.e3();
  res.e3() = qa.e0() * qb.e3() + qa.e3() * qb.e0() - qa.e2() * qb.e1() + qa.e1() * qb.e2();
  return (res);
}
// Get the quaternion from an angle of rotation and an axis, defined in _abs_ coords.
// The axis is supposed to be fixed, i.e. it is constant during rotation.
// The 'axis' vector must be normalized.
ToraQuaternion<double> Q_from_AngAxis(double angle, const ToraVector3<double>& axis) {
  ToraQuaternion<double> quat;
  double halfang;
  double sinhalf;

  halfang = (angle * 0.5);
  sinhalf = sin(halfang);

  quat.e0() = cos(halfang);
  quat.e1() = axis.x() * sinhalf;
  quat.e2() = axis.y() * sinhalf;
  quat.e3() = axis.z() * sinhalf;
  return (quat);
}

// Get the quaternion from a source vector and a destination vector which specifies
// the rotation from one to the other.  The vectors do not need to be normalized.
ToraQuaternion<double> Q_from_Vect_to_Vect(const ToraVector3<double>& fr_vect, const ToraVector3<double>& to_vect) {
  const double ANGLE_TOLERANCE = 1e-6;
  ToraQuaternion<double> quat;
  double halfang;
  double sinhalf;
  ToraVector3<double> axis;

  double lenXlen = fr_vect.Length() * to_vect.Length();
  axis = fr_vect % to_vect;
  double sinangle = Clamp(axis.Length() / lenXlen, -1.0, +1.0);
  double cosangle = Clamp(fr_vect ^ to_vect / lenXlen, -1.0, +1.0);

  // Consider three cases: Parallel, Opposite, non-collinear
  if (std::abs(sinangle) == 0.0 && cosangle > 0) {
    // fr_vect & to_vect are parallel
    quat.e0() = 1.0;
    quat.e1() = 0.0;
    quat.e2() = 0.0;
    quat.e3() = 0.0;
  } else if (std::abs(sinangle) < ANGLE_TOLERANCE && cosangle < 0) {
    // fr_vect & to_vect are opposite, i.e. ~180 deg apart
    axis = fr_vect.GetOrthogonalVector() + (-to_vect).GetOrthogonalVector();
    axis.Normalize();
    quat.e0() = 0.0;
    quat.e1() = Clamp(axis.x(), -1.0, +1.0);
    quat.e2() = Clamp(axis.y(), -1.0, +1.0);
    quat.e3() = Clamp(axis.z(), -1.0, +1.0);
  } else {
    // fr_vect & to_vect are not co-linear case
    axis.Normalize();
    halfang = 0.5 * Atan2(cosangle, sinangle);
    sinhalf = sin(halfang);

    quat.e0() = cos(halfang);
    quat.e1() = sinhalf * axis.x();
    quat.e2() = sinhalf * axis.y();
    quat.e3() = sinhalf * axis.z();
  }
  return (quat);
}

ToraQuaternion<double> Q_from_AngZ(double angleZ) { return Q_from_AngAxis(angleZ, VECT_Z); }
ToraQuaternion<double> Q_from_AngX(double angleX) { return Q_from_AngAxis(angleX, VECT_X); }
ToraQuaternion<double> Q_from_AngY(double angleY) { return Q_from_AngAxis(angleY, VECT_Y); }

ToraQuaternion<double> Q_from_NasaAngles(const ToraVector3<double>& mang) {
  ToraQuaternion<double> mq;
  double c1 = cos(mang.z() / 2);
  double s1 = sin(mang.z() / 2);
  double c2 = cos(mang.x() / 2);
  double s2 = sin(mang.x() / 2);
  double c3 = cos(mang.y() / 2);
  double s3 = sin(mang.y() / 2);
  double c1c2 = c1 * c2;
  double s1s2 = s1 * s2;
  mq.e0() = c1c2 * c3 + s1s2 * s3;
  mq.e1() = c1c2 * s3 - s1s2 * c3;
  mq.e2() = c1 * s2 * c3 + s1 * c2 * s3;
  mq.e3() = s1 * c2 * c3 - c1 * s2 * s3;
  return mq;
}

ToraVector3<double> Q_to_NasaAngles(const ToraQuaternion<double>& q1) {
  ToraVector3<double> mnasa;
  double sqw = q1.e0() * q1.e0();
  double sqx = q1.e1() * q1.e1();
  double sqy = q1.e2() * q1.e2();
  double sqz = q1.e3() * q1.e3();
  // heading
  mnasa.z() = atan2(2.0 * (q1.e1() * q1.e2() + q1.e3() * q1.e0()), (sqx - sqy - sqz + sqw));
  // bank
  mnasa.y() = atan2(2.0 * (q1.e2() * q1.e3() + q1.e1() * q1.e0()), (-sqx - sqy + sqz + sqw));
  // attitude
  mnasa.x() = asin(-2.0 * (q1.e1() * q1.e3() - q1.e2() * q1.e0()));
  return mnasa;
}

ToraQuaternion<double> Q_from_Euler123(const ToraVector3<double>& ang) {
  ToraQuaternion<double> q;
  double t0 = cos(ang.z() * 0.5);
  double t1 = sin(ang.z() * 0.5);
  double t2 = cos(ang.x() * 0.5);
  double t3 = sin(ang.x() * 0.5);
  double t4 = cos(ang.y() * 0.5);
  double t5 = sin(ang.y() * 0.5);

  q.e0() = t0 * t2 * t4 + t1 * t3 * t5;
  q.e1() = t0 * t3 * t4 - t1 * t2 * t5;
  q.e2() = t0 * t2 * t5 + t1 * t3 * t4;
  q.e3() = t1 * t2 * t4 - t0 * t3 * t5;

  return q;
}

ToraVector3<double> Q_to_Euler123(const ToraQuaternion<double>& mq) {
  ToraVector3<double> euler;
  double sq0 = mq.e0() * mq.e0();
  double sq1 = mq.e1() * mq.e1();
  double sq2 = mq.e2() * mq.e2();
  double sq3 = mq.e3() * mq.e3();
  // roll
  euler.x() = atan2(2 * (mq.e2() * mq.e3() + mq.e0() * mq.e1()), sq3 - sq2 - sq1 + sq0);
  // pitch
  euler.y() = -asin(2 * (mq.e1() * mq.e3() - mq.e0() * mq.e2()));
  // yaw
  euler.z() = atan2(2 * (mq.e1() * mq.e2() + mq.e3() * mq.e0()), sq1 + sq0 - sq3 - sq2);

  return euler;
}

void Q_to_AngAxis(const ToraQuaternion<double>& quat, double& angle, ToraVector3<double>& axis) {
  if (std::abs(quat.e0()) < 0.99999999) {
    double arg = acos(quat.e0());
    double invsine = 1 / sin(arg);
    ToraVector3<double> vtemp;
    vtemp.x() = invsine * quat.e1();
    vtemp.y() = invsine * quat.e2();
    vtemp.z() = invsine * quat.e3();
    angle = 2 * arg;
    axis = Vnorm(vtemp);
  } else {
    axis.x() = 1;
    axis.y() = 0;
    axis.z() = 0;
    angle = 0;
  }
}

// Check if two quaternions are equal
bool Qequal(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb) { return qa == qb; }

// Check if quaternion is not null
bool Qnotnull(const ToraQuaternion<double>& qa) {
  return (qa.e0() != 0) || (qa.e1() != 0) || (qa.e2() != 0) || (qa.e3() != 0);
}

}  // namespace Tora