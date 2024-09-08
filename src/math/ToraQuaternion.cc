#include "ToraQuaternion.h"

namespace Tora {

const ToraQuaternion<double> QNULL(0., 0., 0., 0.);
const ToraQuaternion<double> QUNIT(1., 0., 0., 0.);

const ToraQuaternion<double> Q_ROTATE_Y_TO_X(SQRT_2, 0, 0, -SQRT_2);
const ToraQuaternion<double> Q_ROTATE_Y_TO_Z(SQRT_2, SQRT_2, 0, 0);
const ToraQuaternion<double> Q_ROTATE_X_TO_Y(SQRT_2, 0, 0, SQRT_2);
const ToraQuaternion<double> Q_ROTATE_X_TO_Z(SQRT_2, 0, -SQRT_2, 0);
const ToraQuaternion<double> Q_ROTATE_Z_TO_Y(SQRT_2, -SQRT_2, 0, 0);
const ToraQuaternion<double> Q_ROTATE_Z_TO_X(SQRT_2, 0, SQRT_2, 0);

const ToraQuaternion<double> Q_FLIP_AROUND_X(0., 1., 0., 0.);
const ToraQuaternion<double> Q_FLIP_AROUND_Y(0., 0., 1., 0.);
const ToraQuaternion<double> Q_FLIP_AROUND_Z(0., 0., 0., 1.);

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ToraQuaternion<Real>::ToraQuaternion() {
  m_data[0] = 0;
  m_data[1] = 0;
  m_data[2] = 0;
  m_data[3] = 0;
}

template <class Real>
inline ToraQuaternion<Real>::ToraQuaternion(Real e0, Real e1, Real e2, Real e3) {
  m_data[0] = e0;
  m_data[1] = e1;
  m_data[2] = e2;
  m_data[3] = e3;
}

template <class Real>
inline ToraQuaternion<Real>::ToraQuaternion(Real s, const ToraVector3<Real>& v) {
  m_data[0] = s;
  m_data[1] = v.x();
  m_data[2] = v.y();
  m_data[3] = v.z();
}

template <class Real>
inline ToraQuaternion<Real>::ToraQuaternion(const ToraQuaternion<Real>& other) {
  m_data[0] = other.m_data[0];
  m_data[1] = other.m_data[1];
  m_data[2] = other.m_data[2];
  m_data[3] = other.m_data[3];
}

template <class Real>
template <class RealB>
inline ToraQuaternion<Real>::ToraQuaternion(const ToraQuaternion<RealB>& other) {
  m_data[0] = static_cast<Real>(other.m_data[0]);
  m_data[1] = static_cast<Real>(other.m_data[1]);
  m_data[2] = static_cast<Real>(other.m_data[2]);
  m_data[3] = static_cast<Real>(other.m_data[3]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ToraQuaternion<Real>::operator[](unsigned index) {
  assert(index < 4);
  return m_data[index];
}

template <class Real>
inline const Real& ToraQuaternion<Real>::operator[](unsigned index) const {
  assert(index < 4);
  return m_data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator=(const ToraQuaternion<Real>& other) {
  if (&other == this) return *this;
  m_data[0] = other.m_data[0];
  m_data[1] = other.m_data[1];
  m_data[2] = other.m_data[2];
  m_data[3] = other.m_data[3];
  return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator+() const {
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator-() const {
  return ToraQuaternion<Real>(-m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

// -----------------------------------------------------------------------------
// Arithmetic & quaternion operations

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator!() const {
  return ToraQuaternion<Real>(m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator+(const ToraQuaternion<Real>& other) const {
  return ToraQuaternion<Real>(m_data[0] + other.m_data[0], m_data[1] + other.m_data[1], m_data[2] + other.m_data[2],
                              m_data[3] + other.m_data[3]);
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator+=(const ToraQuaternion<Real>& other) {
  m_data[0] += other.m_data[0];
  m_data[1] += other.m_data[1];
  m_data[2] += other.m_data[2];
  m_data[3] += other.m_data[3];
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator-(const ToraQuaternion<Real>& other) const {
  return ToraQuaternion<Real>(m_data[0] - other.m_data[0], m_data[1] - other.m_data[1], m_data[2] - other.m_data[2],
                              m_data[3] - other.m_data[3]);
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator-=(const ToraQuaternion<Real>& other) {
  m_data[0] -= other.m_data[0];
  m_data[1] -= other.m_data[1];
  m_data[2] -= other.m_data[2];
  m_data[3] -= other.m_data[3];
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator*(const ToraQuaternion<Real>& other) const {
  ToraQuaternion<Real> q;
  q.Cross(*this, other);
  return q;
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator*=(const ToraQuaternion<Real>& other) {
  this->Cross(*this, other);
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator>>(const ToraQuaternion<Real>& other) const {
  ToraQuaternion<Real> q;
  q.Cross(other, *this);
  return q;
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator>>=(const ToraQuaternion<Real>& other) {
  this->Cross(other, *this);
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator*(Real s) const {
  return ToraQuaternion<Real>(m_data[0] * s, m_data[1] * s, m_data[2] * s, m_data[3] * s);
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator*=(Real s) {
  m_data[0] *= s;
  m_data[1] *= s;
  m_data[2] *= s;
  m_data[3] *= s;
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator/(const ToraQuaternion<Real>& other) const {
  return ToraQuaternion<Real>(m_data[0] / other.m_data[0], m_data[1] / other.m_data[1], m_data[2] / other.m_data[2],
                              m_data[3] / other.m_data[3]);
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator/=(const ToraQuaternion<Real>& other) {
  m_data[0] /= other.m_data[0];
  m_data[1] /= other.m_data[1];
  m_data[2] /= other.m_data[2];
  m_data[3] /= other.m_data[3];
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator/(Real s) const {
  Real oos = 1 / s;
  return ToraQuaternion<Real>(m_data[0] * oos, m_data[1] * oos, m_data[2] * oos, m_data[3] * oos);
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator/=(Real s) {
  Real oos = 1 / s;
  m_data[0] *= oos;
  m_data[1] *= oos;
  m_data[2] *= oos;
  m_data[3] *= oos;
  return *this;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::operator%(const ToraQuaternion<Real>& other) const {
  ToraQuaternion<Real> q;
  q.Cross(*this, other);
  return q;
}

template <class Real>
inline ToraQuaternion<Real>& ToraQuaternion<Real>::operator%=(const ToraQuaternion<Real>& other) {
  this->Cross(*this, other);
  return *this;
}

template <class Real>
inline Real ToraQuaternion<Real>::operator^(const ToraQuaternion<Real>& other) const {
  return this->Dot(other);
}

// -----------------------------------------------------------------------------
// Comparison operations

template <class Real>
inline bool ToraQuaternion<Real>::operator<=(const ToraQuaternion<Real>& other) const {
  return m_data[0] <= other.m_data[0] && m_data[1] <= other.m_data[1] && m_data[2] <= other.m_data[2] &&
         m_data[3] <= other.m_data[3];
}

template <class Real>
inline bool ToraQuaternion<Real>::operator>=(const ToraQuaternion<Real>& other) const {
  return m_data[0] >= other.m_data[0] && m_data[1] >= other.m_data[1] && m_data[2] >= other.m_data[2] &&
         m_data[3] >= other.m_data[3];
}

template <class Real>
inline bool ToraQuaternion<Real>::operator<(const ToraQuaternion<Real>& other) const {
  return m_data[0] < other.m_data[0] && m_data[1] < other.m_data[1] && m_data[2] < other.m_data[2] &&
         m_data[3] < other.m_data[3];
}

template <class Real>
inline bool ToraQuaternion<Real>::operator>(const ToraQuaternion<Real>& other) const {
  return m_data[0] > other.m_data[0] && m_data[1] > other.m_data[1] && m_data[2] > other.m_data[2] &&
         m_data[3] > other.m_data[3];
}

template <class Real>
inline bool ToraQuaternion<Real>::operator==(const ToraQuaternion<Real>& other) const {
  return other.m_data[0] == m_data[0] && other.m_data[1] == m_data[1] && other.m_data[2] == m_data[2] &&
         other.m_data[3] == m_data[3];
}

template <class Real>
inline bool ToraQuaternion<Real>::operator!=(const ToraQuaternion<Real>& other) const {
  return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ToraQuaternion<Real>::Set(Real e0, Real e1, Real e2, Real e3) {
  m_data[0] = e0;
  m_data[1] = e1;
  m_data[2] = e2;
  m_data[3] = e3;
}

template <class Real>
inline void ToraQuaternion<Real>::Set(const ToraQuaternion<Real>& q) {
  m_data[0] = q.m_data[0];
  m_data[1] = q.m_data[1];
  m_data[2] = q.m_data[2];
  m_data[3] = q.m_data[3];
}

template <class Real>
inline void ToraQuaternion<Real>::SetNull() {
  m_data[0] = 0;
  m_data[1] = 0;
  m_data[2] = 0;
  m_data[3] = 0;
}

template <class Real>
inline void ToraQuaternion<Real>::SetUnit() {
  m_data[0] = 1;
  m_data[1] = 0;
  m_data[2] = 0;
  m_data[3] = 0;
}

template <class Real>
inline void ToraQuaternion<Real>::SetScalar(Real s) {
  m_data[0] = s;
}

template <class Real>
inline void ToraQuaternion<Real>::SetVector(const ToraVector3<Real>& v) {
  m_data[1] = v.x();
  m_data[2] = v.y();
  m_data[3] = v.z();
}

template <class Real>
inline bool ToraQuaternion<Real>::Equals(const ToraQuaternion<Real>& other) const {
  return (other.m_data[0] == m_data[0]) && (other.m_data[1] == m_data[1]) && (other.m_data[2] == m_data[2]) &&
         (other.m_data[3] == m_data[3]);
}

template <class Real>
inline bool ToraQuaternion<Real>::Equals(const ToraQuaternion<Real>& other, Real tol) const {
  return (std::abs(other.m_data[0] - m_data[0]) < tol) && (std::abs(other.m_data[1] - m_data[1]) < tol) &&
         (std::abs(other.m_data[2] - m_data[2]) < tol) && (std::abs(other.m_data[3] - m_data[3]) < tol);
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::GetVector() const {
  return ToraVector3<Real>(m_data[1], m_data[2], m_data[3]);
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::GetXaxis() const {
  return ToraVector3<Real>((m_data[0] * m_data[0] + m_data[1] * m_data[1]) * 2 - 1,
                           (m_data[1] * m_data[2] + m_data[0] * m_data[3]) * 2,
                           (m_data[1] * m_data[3] - m_data[0] * m_data[2]) * 2);
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::GetYaxis() const {
  return ToraVector3<Real>((m_data[1] * m_data[2] - m_data[0] * m_data[3]) * 2,
                           (m_data[0] * m_data[0] + m_data[2] * m_data[2]) * 2 - 1,
                           (m_data[2] * m_data[3] + m_data[0] * m_data[1]) * 2);
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::GetZaxis() const {
  return ToraVector3<Real>((m_data[1] * m_data[3] + m_data[0] * m_data[2]) * 2,
                           (m_data[2] * m_data[3] - m_data[0] * m_data[1]) * 2,
                           (m_data[0] * m_data[0] + m_data[3] * m_data[3]) * 2 - 1);
}

template <class Real>
inline Real ToraQuaternion<Real>::Length() const {
  return sqrt(Length2());
}

template <class Real>
inline Real ToraQuaternion<Real>::Length2() const {
  return this->Dot(*this);
}

template <class Real>
inline void ToraQuaternion<Real>::Add(const ToraQuaternion<Real>& A, const ToraQuaternion<Real>& B) {
  m_data[0] = A.m_data[0] + B.m_data[0];
  m_data[1] = A.m_data[1] + B.m_data[1];
  m_data[2] = A.m_data[2] + B.m_data[2];
  m_data[3] = A.m_data[3] + B.m_data[3];
}

template <class Real>
inline void ToraQuaternion<Real>::Sub(const ToraQuaternion<Real>& A, const ToraQuaternion<Real>& B) {
  m_data[0] = A.m_data[0] - B.m_data[0];
  m_data[1] = A.m_data[1] - B.m_data[1];
  m_data[2] = A.m_data[2] - B.m_data[2];
  m_data[3] = A.m_data[3] - B.m_data[3];
}

template <class Real>
inline void ToraQuaternion<Real>::Cross(const ToraQuaternion<Real>& qa, const ToraQuaternion<Real>& qb) {
  Real w = qa.m_data[0] * qb.m_data[0] - qa.m_data[1] * qb.m_data[1] - qa.m_data[2] * qb.m_data[2] -
           qa.m_data[3] * qb.m_data[3];
  Real x = qa.m_data[0] * qb.m_data[1] + qa.m_data[1] * qb.m_data[0] - qa.m_data[3] * qb.m_data[2] +
           qa.m_data[2] * qb.m_data[3];
  Real y = qa.m_data[0] * qb.m_data[2] + qa.m_data[2] * qb.m_data[0] + qa.m_data[3] * qb.m_data[1] -
           qa.m_data[1] * qb.m_data[3];
  Real z = qa.m_data[0] * qb.m_data[3] + qa.m_data[3] * qb.m_data[0] - qa.m_data[2] * qb.m_data[1] +
           qa.m_data[1] * qb.m_data[2];
  m_data[0] = w;
  m_data[1] = x;
  m_data[2] = y;
  m_data[3] = z;
}

template <class Real>
inline Real ToraQuaternion<Real>::Dot(const ToraQuaternion<Real>& B) const {
  return (m_data[0] * B.m_data[0]) + (m_data[1] * B.m_data[1]) + (m_data[2] * B.m_data[2]) + (m_data[3] * B.m_data[3]);
}

template <class Real>
inline void ToraQuaternion<Real>::Mul(const ToraQuaternion<Real>& A, Real s) {
  m_data[0] = A.m_data[0] * s;
  m_data[1] = A.m_data[1] * s;
  m_data[2] = A.m_data[2] * s;
  m_data[3] = A.m_data[3] * s;
}

template <class Real>
inline void ToraQuaternion<Real>::Scale(Real s) {
  m_data[0] *= s;
  m_data[1] *= s;
  m_data[2] *= s;
  m_data[3] *= s;
}

template <class Real>
inline bool ToraQuaternion<Real>::Normalize() {
  Real length = this->Length();
  if (length < std::numeric_limits<Real>::min()) {
    m_data[0] = 1;
    m_data[1] = 0;
    m_data[2] = 0;
    m_data[3] = 0;
    return false;
  }
  this->Scale(1 / length);
  return true;
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::GetNormalized() const {
  ToraQuaternion<Real> q(*this);
  q.Normalize();
  return q;
}

template <class Real>
inline void ToraQuaternion<Real>::Conjugate(const ToraQuaternion<Real>& A) {
  m_data[0] = +A.m_data[0];
  m_data[1] = -A.m_data[1];
  m_data[2] = -A.m_data[2];
  m_data[3] = -A.m_data[3];
}

template <class Real>
inline void ToraQuaternion<Real>::Conjugate() {
  Conjugate(*this);
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::GetConjugate() const {
  return ToraQuaternion<Real>(m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

template <class Real>
inline ToraQuaternion<Real> ToraQuaternion<Real>::GetInverse() const {
  ToraQuaternion<Real> invq = this->GetConjugate();
  invq.Scale(1 / this->Length2());
  return invq;
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::Rotate(const ToraVector3<Real>& A) const {
  Real e0e0 = m_data[0] * m_data[0];
  Real e1e1 = m_data[1] * m_data[1];
  Real e2e2 = m_data[2] * m_data[2];
  Real e3e3 = m_data[3] * m_data[3];
  Real e0e1 = m_data[0] * m_data[1];
  Real e0e2 = m_data[0] * m_data[2];
  Real e0e3 = m_data[0] * m_data[3];
  Real e1e2 = m_data[1] * m_data[2];
  Real e1e3 = m_data[1] * m_data[3];
  Real e2e3 = m_data[2] * m_data[3];
  return ToraVector3<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                           ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                           ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::RotateBack(const ToraVector3<Real>& A) const {
  Real e0e0 = +m_data[0] * m_data[0];
  Real e1e1 = +m_data[1] * m_data[1];
  Real e2e2 = +m_data[2] * m_data[2];
  Real e3e3 = +m_data[3] * m_data[3];
  Real e0e1 = -m_data[0] * m_data[1];
  Real e0e2 = -m_data[0] * m_data[2];
  Real e0e3 = -m_data[0] * m_data[3];
  Real e1e2 = +m_data[1] * m_data[2];
  Real e1e3 = +m_data[1] * m_data[3];
  Real e2e3 = +m_data[2] * m_data[3];
  return ToraVector3<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                           ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                           ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline void ToraQuaternion<Real>::Q_from_Rotv(const ToraVector3<Real>& angle_axis) {
  Real theta_squared = angle_axis.Length2();
  // For non-zero rotation:
  if (theta_squared > 1e-30) {
    Real theta = sqrt(theta_squared);
    Real half_theta = theta / 2;
    Real k = sin(half_theta) / theta;
    m_data[0] = cos(half_theta);
    m_data[1] = angle_axis.x() * k;
    m_data[2] = angle_axis.y() * k;
    m_data[3] = angle_axis.z() * k;
  } else {
    // For almost zero rotation:
    Real k(0.5);
    m_data[0] = Real(1.0);
    m_data[1] = angle_axis.x() * k;
    m_data[2] = angle_axis.y() * k;
    m_data[3] = angle_axis.z() * k;
  }
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::Q_to_Rotv() {
  ToraVector3<Real> angle_axis;
  Real sin_squared = m_data[1] * m_data[1] + m_data[2] * m_data[2] + m_data[3] * m_data[3];
  // For non-zero rotation
  if (sin_squared > 1e-30) {
    Real sin_theta = sqrt(sin_squared);
    Real k = 2 * atan2(sin_theta, m_data[0]) / sin_theta;
    angle_axis.x() = m_data[1] * k;
    angle_axis.y() = m_data[2] * k;
    angle_axis.z() = m_data[3] * k;
  } else {
    // For almost zero rotation
    Real k(2.0);
    angle_axis.x() = m_data[1] * k;
    angle_axis.y() = m_data[2] * k;
    angle_axis.z() = m_data[3] * k;
  }
  return angle_axis;
}

template <class Real>
inline void ToraQuaternion<Real>::Q_from_AngAxis(Real angle, const ToraVector3<Real>& axis) {
  Real halfang = (angle / 2);
  Real sinhalf = sin(halfang);
  m_data[0] = cos(halfang);
  m_data[1] = axis.x() * sinhalf;
  m_data[2] = axis.y() * sinhalf;
  m_data[3] = axis.z() * sinhalf;
}

template <class Real>
inline void ToraQuaternion<Real>::Q_to_AngAxis(Real& a_angle, ToraVector3<Real>& a_axis) const {
  Real sin_squared = m_data[1] * m_data[1] + m_data[2] * m_data[2] + m_data[3] * m_data[3];
  // For non-zero rotation
  if (sin_squared > 0) {
    Real sin_theta = sqrt(sin_squared);
    a_angle = 2 * atan2(sin_theta, m_data[0]);
    Real k = 1 / sin_theta;
    a_axis.x() = m_data[1] * k;
    a_axis.y() = m_data[2] * k;
    a_axis.z() = m_data[3] * k;
    a_axis.Normalize();
  } else {
    // For almost zero rotation
    a_angle = 0.0;
    a_axis.x() = 1;  // m_data[1] * 2.0;
    a_axis.y() = 0;  // m_data[2] * 2.0;
    a_axis.z() = 0;  // m_data[3] * 2.0;
  }
  // Ensure that angle is always in  [-PI...PI] range
  auto PI = static_cast<Real>(CH_C_PI);
  if (a_angle > PI) {
    a_angle -= 2 * PI;
  } else if (a_angle < -PI) {
    a_angle += 2 * PI;
  }
}

template <class Real>
inline void ToraQuaternion<Real>::Q_from_NasaAngles(const ToraVector3<Real>& ang) {
  Real c1 = cos(ang.z() / 2);
  Real s1 = sin(ang.z() / 2);
  Real c2 = cos(ang.x() / 2);
  Real s2 = sin(ang.x() / 2);
  Real c3 = cos(ang.y() / 2);
  Real s3 = sin(ang.y() / 2);

  Real c1c2 = c1 * c2;
  Real s1s2 = s1 * s2;

  m_data[0] = c1c2 * c3 + s1s2 * s3;
  m_data[1] = c1c2 * s3 - s1s2 * c3;
  m_data[2] = c1 * s2 * c3 + s1 * c2 * s3;
  m_data[3] = s1 * c2 * c3 - c1 * s2 * s3;
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::Q_to_NasaAngles() {
  ToraVector3<Real> nasa;
  Real sqw = m_data[0] * m_data[0];
  Real sqx = m_data[1] * m_data[1];
  Real sqy = m_data[2] * m_data[2];
  Real sqz = m_data[3] * m_data[3];
  // heading
  nasa.z() = atan2(2 * (m_data[1] * m_data[2] + m_data[3] * m_data[0]), (sqx - sqy - sqz + sqw));
  // bank
  nasa.y() = atan2(2 * (m_data[2] * m_data[3] + m_data[1] * m_data[0]), (-sqx - sqy + sqz + sqw));
  // attitude
  nasa.x() = asin(-2 * (m_data[1] * m_data[3] - m_data[2] * m_data[0]));
  return nasa;
}

template <class Real>
inline void ToraQuaternion<Real>::Q_from_Euler123(const ToraVector3<Real>& ang) {
  // Angles {phi;theta;psi} aka {roll;pitch;yaw}
  Real t0 = cos(ang.z() * Real(0.5));
  Real t1 = sin(ang.z() * Real(0.5));
  Real t2 = cos(ang.x() * Real(0.5));
  Real t3 = sin(ang.x() * Real(0.5));
  Real t4 = cos(ang.y() * Real(0.5));
  Real t5 = sin(ang.y() * Real(0.5));

  m_data[0] = t0 * t2 * t4 + t1 * t3 * t5;
  m_data[1] = t0 * t3 * t4 - t1 * t2 * t5;
  m_data[2] = t0 * t2 * t5 + t1 * t3 * t4;
  m_data[3] = t1 * t2 * t4 - t0 * t3 * t5;
}

template <class Real>
inline ToraVector3<Real> ToraQuaternion<Real>::Q_to_Euler123() {
  // Angles {phi;theta;psi} aka {roll;pitch;yaw} rotation XYZ
  ToraVector3<Real> euler;
  Real sq0 = m_data[0] * m_data[0];
  Real sq1 = m_data[1] * m_data[1];
  Real sq2 = m_data[2] * m_data[2];
  Real sq3 = m_data[3] * m_data[3];
  // roll
  euler.x() = atan2(2 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]), sq3 - sq2 - sq1 + sq0);
  // pitch
  euler.y() = -asin(2 * (m_data[1] * m_data[3] - m_data[0] * m_data[2]));
  // yaw
  euler.z() = atan2(2 * (m_data[1] * m_data[2] + m_data[3] * m_data[0]), sq1 + sq0 - sq3 - sq2);
  return euler;
}

}  // namespace Tora