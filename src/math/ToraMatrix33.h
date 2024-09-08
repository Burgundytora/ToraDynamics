// =============================================================================
// Authors: Tora
// =============================================================================

#ifndef TORA_MATRIX33_H
#define TORA_MATRIX33_H

#include <cmath>

#include "ToraMatrix.h"
#include "ToraQuaternion.h"
#include "ToraVector3.h"

namespace Tora {

/// Definition of a 3x3 fixed size matrix to represent 3D rotations and inertia tensors.
template <typename Real = double>
class ToraMatrix33 : public Eigen::Matrix<Real, 3, 3, Eigen::RowMajor> {
 private:
  // member in father class(Eigen::Matrix).

 public:
  /// Default constructor: uninitialized 3x3 matrix.
  ToraMatrix33() : Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>() {}

  /// Constructor from Eigen expressions.
  template <typename OtherDerived>
  ToraMatrix33(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>(other) {}

  /// Construct a diagonal matrix with the specified value on the diagonal.
  ToraMatrix33(Real val);

  /// Construct a diagonal matrix with the specified values on the diagonal.
  ToraMatrix33(const ToraVector3<Real>& v);

  /// Construct a symmetric 3x3 matrix with the specified vectors for the diagonal and off-digonal elements.
  /// The off-diagonal vector is assumed to contain the elements A(0,1), A(0,2), A(1,2) in this order.
  ToraMatrix33(const ToraVector3<>& diag, const ToraVector3<>& off_diag);

  /// Construct a 3x3 rotation matrix from the given quaternion.
  ToraMatrix33(const ToraQuaternion<Real>& q);

  /// Construct a 3x3 rotation matrix from an angle and a rotation axis.
  /// Note that the axis direction must be normalized.
  ToraMatrix33(Real angle, const ToraVector3<>& axis);

  /// Construct a 3x3 matrix with the given vectors as columns.
  /// If the three vectors are mutually orthogonal unit vectors, the resulting matrix is a rotation matrix.
  ToraMatrix33(const ToraVector3<>& X, const ToraVector3<>& Y, const ToraVector3<>& Z);

  /// This method allows assigning Eigen expressions to ToraMatrix33.
  template <typename OtherDerived>
  ToraMatrix33& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>::operator=(other);
    return *this;
  }

  /// Allows multiplying a 3x3 matrix to other Eigen matrices.
  using Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>::operator*;

  /// Multiply this matrix by a 3d vector.
  ToraVector3<Real> operator*(const ToraVector3<Real>& v) const;

  /// Fill this 3x3 matrix as a rotation matrix, given a unit quaternion.
  void Set_A_quaternion(const ToraQuaternion<Real>& quat);

  /// Fill this 3x3 matrix as a rotation matrix, given three Euler angles.
  void Set_A_Eulero(const ToraVector3<Real>& angles);

  /// Fill this 3x3 matrix as a rotation matrix, given three Cardano angles.
  void Set_A_Cardano(const ToraVector3<Real>& angles);

  /// Fill this 3x3 matrix as a rotation matrix, given the three versors X,Y,Z of the basis.
  void Set_A_axis(const ToraVector3<Real>& X, const ToraVector3<Real>& Y, const ToraVector3<Real>& Z);

  /// Fill this 3x3 matrix as a rotation matrix with the X axis along the provided direction.
  /// Uses the Gram-Schmidt orthonormalization. The optional argument Vsingular, together with Xdir, suggests the XY
  /// plane (as long as Xdir is not too close to lying in that plane, in which case a different direction is
  /// selected).
  void Set_A_Xdir(const ToraVector3<Real>& Xdir,                                   ///< X axis
                  const ToraVector3<Real>& Vsingular = ToraVector3<Real>(0, 1, 0)  ///< suggested Y axis
  );

  /// Return the versor of X axis.
  ToraVector3<Real> Get_A_Xaxis() const;

  /// Return the versor of Y axis.
  ToraVector3<Real> Get_A_Yaxis() const;

  /// Return the versor of Z axis.
  ToraVector3<Real> Get_A_Zaxis() const;

  /// Return the corresponding unit quaternion.
  /// Assumes that this is a rotation matrix.
  ToraQuaternion<Real> Get_A_quaternion() const;

  /// Return the Eulero angles.
  /// Assumes that this is a rotation matrix.
  ToraVector3<Real> Get_A_Eulero() const;

  /// Assuming this matrix is a rotation matrix, get Ax vector.
  ToraVector3<Real> GetAx() const;
};

// -----------------------------------------------------------------------------

/// Multiply a transposed 3x3 matrix with a vector.
template <typename Real>
ToraVector3<Real> operator*(const Eigen::Transpose<Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>>& A,
                            const ToraVector3<Real>& v) {
  return ToraVector3<Real>(A(0, 0) * v.x() + A(0, 1) * v.y() + A(0, 2) * v.z(),
                           A(1, 0) * v.x() + A(1, 1) * v.y() + A(1, 2) * v.z(),
                           A(2, 0) * v.x() + A(2, 1) * v.y() + A(2, 2) * v.z());
}

/// Multiply a transposed const 3x3 matrix with a vector.
template <typename Real>
ToraVector3<Real> operator*(const Eigen::Transpose<const Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>>& A,
                            const ToraVector3<Real>& v) {
  return ToraVector3<Real>(A(0, 0) * v.x() + A(0, 1) * v.y() + A(0, 2) * v.z(),
                           A(1, 0) * v.x() + A(1, 1) * v.y() + A(1, 2) * v.z(),
                           A(2, 0) * v.x() + A(2, 1) * v.y() + A(2, 2) * v.z());
}

/// Return the outer product (a 3x3 matrix) of two vectors.
template <class Real>
ToraMatrix33<Real> TensorProduct(const ToraVector3<Real>& vA, const ToraVector3<Real>& vB) {
  ToraMatrix33<Real> T;
  T(0, 0) = vA.x() * vB.x();
  T(0, 1) = vA.x() * vB.y();
  T(0, 2) = vA.x() * vB.z();
  T(1, 0) = vA.y() * vB.x();
  T(1, 1) = vA.y() * vB.y();
  T(1, 2) = vA.y() * vB.z();
  T(2, 0) = vA.z() * vB.x();
  T(2, 1) = vA.z() * vB.y();
  T(2, 2) = vA.z() * vB.z();
  return T;
}

}  // namespace Tora

#endif
