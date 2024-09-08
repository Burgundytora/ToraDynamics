// =============================================================================
// Authors: Tora
// =============================================================================

#ifndef TORA_QUATERNION_H
#define TORA_QUATERNION_H

#include <algorithm>
#include <cmath>
#include <limits>

#include "ToraConstant.h"
#include "ToraMatrix.h"
#include "ToraVector3.h"

namespace Tora {

/// Definitions of various angle sets for conversions.
enum class AngleSet {
  ANGLE_AXIS,
  EULERO,   ///< sequence: Z - X' - Z''
  CARDANO,  ///< sequence: Z - X' - Y''
  HPB,      ///< sequence:
  RXYZ,     ///< sequence: X - Y' - Z''
  RODRIGUEZ,
  QUATERNION,
};

template <class Real = double>
class ToraQuaternion {
 private:
  /// Data in the order e0, e1, e2, e3
  Real m_data[4];

  /// Declaration of friend classes
  template <typename RealB>
  friend class ToraQuaternion;

 public:
  /// Default constructor.
  /// Note that this constructs a null quaternion {0,0,0,0}, not a {1,0,0,0} unit quaternion.
  ToraQuaternion();

  /// Constructor from four scalars. The first is the real part, others are i,j,k imaginary parts
  ToraQuaternion(Real e0, Real e1, Real e2, Real e3);

  /// Constructor from real part, and vector with i,j,k imaginary part.
  ToraQuaternion(Real s, const ToraVector3<Real>& v);

  /// Copy constructor
  ToraQuaternion(const ToraQuaternion<Real>& other);

  /// Copy constructor with type change.
  template <class RealB>
  ToraQuaternion(const ToraQuaternion<RealB>& other);

  /// Access to components
  Real& e0() { return m_data[0]; }
  Real& e1() { return m_data[1]; }
  Real& e2() { return m_data[2]; }
  Real& e3() { return m_data[3]; }
  const Real& e0() const { return m_data[0]; }
  const Real& e1() const { return m_data[1]; }
  const Real& e2() const { return m_data[2]; }
  const Real& e3() const { return m_data[3]; }

  /// Return const pointer to underlying array storage.
  const Real* data() const { return m_data; }

  // EIGEN INTER-OPERABILITY

  /// Construct a quaternion from an Eigen vector expression.
  template <typename Derived>
  ToraQuaternion(const Eigen::MatrixBase<Derived>& vec,
                 typename std::enable_if<(Derived::MaxRowsAtCompileTime == 1 || Derived::MaxColsAtCompileTime == 1),
                                         Derived>::type* = 0) {
    m_data[0] = vec(0);
    m_data[1] = vec(1);
    m_data[2] = vec(2);
    m_data[3] = vec(3);
  }

  /// View this quaternion as an Eigen vector.
  Eigen::Map<Eigen::Matrix<Real, 4, 1>> eigen() { return Eigen::Map<Eigen::Matrix<Real, 4, 1>>(m_data); }
  Eigen::Map<const Eigen::Matrix<Real, 4, 1>> eigen() const {
    return Eigen::Map<const Eigen::Matrix<Real, 4, 1>>(m_data);
  }

  /// Assign an Eigen vector expression to this quaternion.
  template <typename Derived>
  ToraQuaternion& operator=(const Eigen::MatrixBase<Derived>& vec) {
    m_data[0] = vec(0);
    m_data[1] = vec(1);
    m_data[2] = vec(2);
    m_data[3] = vec(3);
    return *this;
  }

  // SET & GET FUNCTIONS

  /// Sets the four values of the quaternion at once
  void Set(Real e0, Real e1, Real e2, Real e3);

  /// Sets the quaternion as a copy of another quaternion
  void Set(const ToraQuaternion<Real>& q);

  /// Sets the quaternion as a null quaternion
  void SetNull();

  /// Sets the quaternion as a unit quaternion
  void SetUnit();

  /// Sets the scalar part only
  void SetScalar(Real s);

  /// Sets the vectorial part only
  void SetVector(const ToraVector3<Real>& v);

  /// Return true if quaternion is identical to other quaternion
  bool Equals(const ToraQuaternion<Real>& other) const;

  /// Return true if quaternion equals another quaternion, within a tolerance 'tol'
  bool Equals(const ToraQuaternion<Real>& other, Real tol) const;

  /// Gets the vectorial part only
  ToraVector3<Real> GetVector() const;

  /// Get the X axis of a coordsystem, given the quaternion which represents
  /// the alignment of the coordsystem. Note that it is assumed that the
  /// quaternion is already normalized.
  ToraVector3<Real> GetXaxis() const;

  /// Get the Y axis of a coordsystem, given the quaternion which represents
  /// the alignment of the coordsystem. Note that it is assumed that the
  /// quaternion is already normalized.
  ToraVector3<Real> GetYaxis() const;

  /// Get the Z axis of a coordsystem, given the quaternion which represents
  /// the alignment of the coordsystem. Note that it is assumed that the
  /// quaternion is already normalized.
  ToraVector3<Real> GetZaxis() const;

  // QUATERNION NORMS

  /// Compute the euclidean norm of the quaternion, that is its length or magnitude.
  Real Length() const;

  /// Compute the squared euclidean norm of the quaternion.
  Real Length2() const;

  // OPERATORS OVERLOADING

  /// Subscript operator.
  Real& operator[](unsigned index);
  const Real& operator[](unsigned index) const;

  /// Assignment operator: copy from another quaternion.
  ToraQuaternion<Real>& operator=(const ToraQuaternion<Real>& other);

  /// Operator for sign change.
  ToraQuaternion<Real> operator+() const;
  ToraQuaternion<Real> operator-() const;

  /// Operator for making a conjugate quaternion (the original is not changed).
  /// A conjugate quaternion has the vectorial part with changed sign.
  ToraQuaternion<Real> operator!() const;

  /// Operator for quaternion sum.
  ToraQuaternion<Real> operator+(const ToraQuaternion<Real>& other) const;
  ToraQuaternion<Real>& operator+=(const ToraQuaternion<Real>& other);

  /// Operator for quaternion difference.
  ToraQuaternion<Real> operator-(const ToraQuaternion<Real>& other) const;
  ToraQuaternion<Real>& operator-=(const ToraQuaternion<Real>& other);

  // NOTE
  // The following * and *= operators had a different behaviour prior to 13/9/2014,
  // but we assume no one used * *= in that previous form (element-by-element product).
  // Now * operator will be used for classical quaternion product, as the old % operator.
  // This is to be more consistent with the * operator for ChFrames etc.

  /// Operator for quaternion product: A*B means the typical quaternion product.
  /// Notes:
  /// - since unit quaternions can represent rotations, the product can represent a
  ///   concatenation of rotations as:
  ///        frame_rotation_2to0 = frame_rotation_1to0 * frame_rotation_2to1
  /// - pay attention to operator low precedence (see C++ precedence rules!)
  /// - quaternion product is not commutative.
  ToraQuaternion<Real> operator*(const ToraQuaternion<Real>& other) const;

  /// Operator for quaternion product and assignment:
  /// A*=B means A'=A*B, with typical quaternion product.
  /// Notes:
  /// - since unit quaternions can represent rotations, the product can represent a
  ///   post-concatenation of a rotation in a kinematic chain.
  /// - quaternion product is not commutative.
  ToraQuaternion<Real>& operator*=(const ToraQuaternion<Real>& other);

  /// Operator for 'specular' quaternion product: A>>B = B*A.
  /// Notes:
  /// - since unit quaternions can represent rotations, the product can represent a
  ///   concatenation of rotations as:
  ///       frame_rotation_2to0 = frame_rotation_2to1 >> frame_rotation_1to0
  /// - pay attention to operator low precedence (see C++ precedence rules!)
  /// - quaternion product is not commutative.
  ToraQuaternion<Real> operator>>(const ToraQuaternion<Real>& other) const;

  /// Operator for quaternion 'specular' product and assignment:
  /// A>>=B means A'=A>>B, or A'=B*A with typical quaternion product.
  /// Notes:
  /// - since unit quaternions can represent rotations, the product can represent a
  ///   pre-concatenation of a rotation in a kinematic chain.
  /// - quaternion product is not commutative.
  ToraQuaternion<Real>& operator>>=(const ToraQuaternion<Real>& other);

  // Operator for scaling the quaternion by a scalar value, as q*s.
  ToraQuaternion<Real> operator*(Real s) const;
  ToraQuaternion<Real>& operator*=(Real s);

  /// Operator for element-wise division.
  /// Note that this is NOT the quaternion division operation.
  ToraQuaternion<Real> operator/(const ToraQuaternion<Real>& other) const;
  ToraQuaternion<Real>& operator/=(const ToraQuaternion<Real>& other);

  /// Operator for scaling the quaternion by inverse of a scalar value, as q/s.
  ToraQuaternion<Real> operator/(Real s) const;
  ToraQuaternion<Real>& operator/=(Real s);

  /// Operator for quaternion product: A%B means the typical quaternion product AxB.
  /// Note: DEPRECATED, use the * operator instead.
  ToraQuaternion<Real> operator%(const ToraQuaternion<Real>& other) const;
  ToraQuaternion<Real>& operator%=(const ToraQuaternion<Real>& other);

  /// Operator for dot product: A^B means the scalar dot-product A*B.
  /// Note: pay attention to operator low precedence (see C++ precedence rules!)
  Real operator^(const ToraQuaternion<Real>& other) const;

  /// Component-wise comparison operators.
  bool operator<=(const ToraQuaternion<Real>& other) const;
  bool operator>=(const ToraQuaternion<Real>& other) const;
  bool operator<(const ToraQuaternion<Real>& other) const;
  bool operator>(const ToraQuaternion<Real>& other) const;
  bool operator==(const ToraQuaternion<Real>& other) const;
  bool operator!=(const ToraQuaternion<Real>& other) const;

  // FUNCTIONS

  /// Set this quaternion to the sum of A and B: this = A + B.
  void Add(const ToraQuaternion<Real>& A, const ToraQuaternion<Real>& B);

  /// Set this quaternion to the difference of A and B: this = A - B.
  void Sub(const ToraQuaternion<Real>& A, const ToraQuaternion<Real>& B);

  /// Set this quaternion to the quaternion product of the two quaternions A and B,
  /// following the classic Hamilton rule:  this = AxB.
  /// This is the true, typical quaternion product. It is NOT commutative.
  void Cross(const ToraQuaternion<Real>& qa, const ToraQuaternion<Real>& qb);

  /// Return the dot product with another quaternion: result = this ^ B.
  Real Dot(const ToraQuaternion<Real>& B) const;

  /// Set this quaternion to the product of a quaternion A and scalar s: this = A * s.
  void Mul(const ToraQuaternion<Real>& A, Real s);

  /// Scale this quaternion by a scalar: this *= s.
  void Scale(Real s);

  /// Normalize this quaternion in place, so that its euclidean length is 1.
  /// Return false if the original quaternion had zero length (in which case the quaternion
  /// is set to [1,0,0,0]) and return true otherwise.
  bool Normalize();

  /// Return a normalized copy of this quaternion, with euclidean length = 1.
  /// Not to be confused with Normalize() which normalizes in place.
  ToraQuaternion<Real> GetNormalized() const;

  /// Set this quaternion to the conjugate of the A quaternion.
  void Conjugate(const ToraQuaternion<Real>& A);

  /// Conjugate this quaternion in place (its vectorial part changes sign).
  void Conjugate();

  /// Return a conjugated version of this quaternion.
  ToraQuaternion<Real> GetConjugate() const;

  /// Return the inverse of this quaternion.
  ToraQuaternion<Real> GetInverse() const;

  // TRANSFORMATIONS

  /// Rotate the vector A on the basis of this quaternion: res=p*[0,A]*p'
  /// (speed-optimized version). Endomorphism assumes p is already normalized.
  ToraVector3<Real> Rotate(const ToraVector3<Real>& A) const;

  /// Rotate the vector A on the basis of conjugate of this quaternion: res=p'*[0,A]*p
  /// (speed-optimized version).  Endomorphism assumes p is already normalized.
  ToraVector3<Real> RotateBack(const ToraVector3<Real>& A) const;

  // CONVERSIONS

  /// Set the quaternion from a rotation vector (ie. a 3D axis of rotation with length as angle of rotation)
  /// defined in absolute coords.
  /// If you need distinct axis and angle, use Q_from_AngAxis().
  void Q_from_Rotv(const ToraVector3<Real>& angle_axis);

  /// Get the rotation vector (ie. a 3D axis of rotation with length as angle of rotation) from a quaternion.
  /// If you need distinct axis and angle, use rather Q_to_AngAxis().
  ToraVector3<Real> Q_to_Rotv();

  /// Set the quaternion from an angle of rotation and an axis, defined in absolute coords.
  /// The axis is supposed to be fixed, i.e. it is constant during rotation!
  /// NOTE, axis must be normalized!
  /// If you need directly the rotation vector=axis * angle, use Q_from_Rotv().
  void Q_from_AngAxis(Real angle, const ToraVector3<Real>& axis);

  /// Set the quaternion from an angle of rotation about X axis.
  void Q_from_AngX(Real angleX) { Q_from_AngAxis(angleX, ToraVector3<Real>(1, 0, 0)); }

  /// Set the quaternion from an angle of rotation about Y axis.
  void Q_from_AngY(Real angleY) { Q_from_AngAxis(angleY, ToraVector3<Real>(0, 1, 0)); }

  /// Set the quaternion from an angle of rotation about Z axis.
  void Q_from_AngZ(Real angleZ) { Q_from_AngAxis(angleZ, ToraVector3<Real>(0, 0, 1)); }

  /// Convert the quaternion to an angle of rotation and an axis, defined in absolute coords.
  /// Resulting angle and axis must be passed as parameters.
  /// Note that angle is in [-PI....+PI] range. Also remember  (angle, axis) is the same of (-angle,-axis).
  /// If you need directly the rotation vector=axis * angle, use Q_to_Rotv().
  void Q_to_AngAxis(Real& a_angle, ToraVector3<Real>& a_axis) const;

  /// Set the quaternion from three angles (NASA angle set) heading, bank, and attitude.
  void Q_from_NasaAngles(const ToraVector3<Real>& ang);

  /// Convert the quaternion to three angles (NASA angle set) heading, bank and attitude.
  ToraVector3<Real> Q_to_NasaAngles();

  /// Set the quaternion from three angles (Euler Sequence 123) roll, pitch, and yaw.
  void Q_from_Euler123(const ToraVector3<Real>& ang);

  /// Convert the quaternion to three angles (Euler Sequence 123) roll, pitch, and yaw.
  ToraVector3<Real> Q_to_Euler123();
};

// -----------------------------------------------------------------------------
// CONSTANTS

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
  auto PI_ = static_cast<Real>(PI);
  if (a_angle > PI_) {
    a_angle -= 2 * PI_;
  } else if (a_angle < -PI_) {
    a_angle += 2 * PI_;
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

// -----------------------------------------------------------------------------
// STATIC QUATERNION MATH OPERATIONS
//
// These functions are here for people which prefer to use static functions
// instead of ToraQuaternion class' member functions.
// NOTE: sometimes a wise adoption of the following functions may give faster
// results than using overloaded operators +/-/* in the quaternion class.

double Qlength(const ToraQuaternion<double>& q);

ToraQuaternion<double> Qadd(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb);

ToraQuaternion<double> Qsub(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb);

ToraQuaternion<double> Qscale(const ToraQuaternion<double>& q, double fact);

/// Return the norm two of the quaternion. Euler's parameters have norm = 1
ToraQuaternion<double> Qnorm(const ToraQuaternion<double>& q);

/// Get the quaternion from an angle of rotation and an axis, defined in _abs_ coords.
/// The axis is supposed to be fixed, i.e. it is constant during rotation.
/// The 'axis' vector must be normalized.
ToraQuaternion<double> Q_from_AngAxis(double angle, const ToraVector3<double>& axis);

/// Get the quaternion from a source vector and a destination vector which specifies
/// the rotation from one to the other.  The vectors do not need to be normalized.
ToraQuaternion<double> Q_from_Vect_to_Vect(const ToraVector3<double>& fr_vect, const ToraVector3<double>& to_vect);

ToraQuaternion<double> Q_from_NasaAngles(const ToraVector3<double>& RxRyRz);

ToraVector3<double> Q_to_NasaAngles(const ToraQuaternion<double>& mq);

ToraQuaternion<double> Q_from_Euler123(const ToraVector3<double>& RxRyRz);

ToraVector3<double> Q_to_Euler123(const ToraQuaternion<double>& mq);

ToraQuaternion<double> Q_from_AngZ(double angleZ);

ToraQuaternion<double> Q_from_AngX(double angleX);

ToraQuaternion<double> Q_from_AngY(double angleY);

void Q_to_AngAxis(const ToraQuaternion<double>& quat, double& angle, ToraVector3<double>& axis);

/// Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
ToraQuaternion<double> Qconjugate(const ToraQuaternion<double>& q);

/// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
ToraQuaternion<double> Qcross(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb);

/// Check if two quaternions are equal.
bool Qequal(const ToraQuaternion<double>& qa, const ToraQuaternion<double>& qb);

/// Check if quaternion is not null.
bool Qnotnull(const ToraQuaternion<double>& qa);

}  // namespace Tora

#endif