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

}  // namespace Tora

#endif