#include "ToraMatrix33.h"

namespace Tora {

// -----------------------------------------------------------------------------
// Implementation of ToraMatrix33 functions
// -----------------------------------------------------------------------------

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(const ToraQuaternion<Real>& q) {
  this->Set_A_quaternion(q);
}

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(Real val) {
  this->setZero();
  this->diagonal().setConstant(val);
}

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(const ToraVector3<Real>& v) {
  this->setZero();
  this->diagonal() = v.eigen();
}

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(const ToraVector3<>& diag, const ToraVector3<>& off_diag) {
  this->diagonal() = diag.eigen();

  (*this)(0, 1) = off_diag.x();
  (*this)(1, 0) = off_diag.x();

  (*this)(0, 2) = off_diag.y();
  (*this)(2, 0) = off_diag.y();

  (*this)(1, 2) = off_diag.z();
  (*this)(2, 1) = off_diag.z();
}

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(Real angle, const ToraVector3<>& axis) {
  ChQuaternion<Real> mr;
  mr.Q_from_AngAxis(angle, axis);
  this->Set_A_quaternion(mr);
}

template <typename Real>
ToraMatrix33<Real>::ToraMatrix33(const ToraVector3<>& X, const ToraVector3<>& Y, const ToraVector3<>& Z) {
  this->Set_A_axis(X, Y, Z);
}

template <typename Real>
ToraVector3<Real> ToraMatrix33<Real>::operator*(const ToraVector3<Real>& v) const {
  return ToraVector3<Real>((*this)(0, 0) * v.x() + (*this)(0, 1) * v.y() + (*this)(0, 2) * v.z(),
                           (*this)(1, 0) * v.x() + (*this)(1, 1) * v.y() + (*this)(1, 2) * v.z(),
                           (*this)(2, 0) * v.x() + (*this)(2, 1) * v.y() + (*this)(2, 2) * v.z());
}

template <typename Real>
inline void ToraMatrix33<Real>::Set_A_quaternion(const ToraQuaternion<Real>& q) {
  Real e0e0 = q.e0() * q.e0();
  Real e1e1 = q.e1() * q.e1();
  Real e2e2 = q.e2() * q.e2();
  Real e3e3 = q.e3() * q.e3();
  Real e0e1 = q.e0() * q.e1();
  Real e0e2 = q.e0() * q.e2();
  Real e0e3 = q.e0() * q.e3();
  Real e1e2 = q.e1() * q.e2();
  Real e1e3 = q.e1() * q.e3();
  Real e2e3 = q.e2() * q.e3();

  (*this)(0, 0) = (e0e0 + e1e1) * 2 - 1;
  (*this)(0, 1) = (e1e2 - e0e3) * 2;
  (*this)(0, 2) = (e1e3 + e0e2) * 2;
  (*this)(1, 0) = (e1e2 + e0e3) * 2;
  (*this)(1, 1) = (e0e0 + e2e2) * 2 - 1;
  (*this)(1, 2) = (e2e3 - e0e1) * 2;
  (*this)(2, 0) = (e1e3 - e0e2) * 2;
  (*this)(2, 1) = (e2e3 + e0e1) * 2;
  (*this)(2, 2) = (e0e0 + e3e3) * 2 - 1;
}

template <typename Real>
inline void ToraMatrix33<Real>::Set_A_Eulero(const ToraVector3<Real>& angles) {
  Real cx = std::cos(angles.x());
  Real cy = std::cos(angles.y());
  Real cz = std::cos(angles.z());
  Real sx = std::sin(angles.x());
  Real sy = std::sin(angles.y());
  Real sz = std::sin(angles.z());

  (*this)(0, 0) = (cz * cx) - (cy * sx * sz);
  (*this)(0, 1) = -(sz * cx) - (cy * sx * cz);
  (*this)(0, 2) = sy * sx;
  (*this)(1, 0) = (cz * sx) + (cy * cx * sz);
  (*this)(1, 1) = -(sz * sx) + (cy * cx * cz);
  (*this)(1, 2) = -sy * cx;
  (*this)(2, 0) = sy * sz;
  (*this)(2, 1) = sy * cz;
  (*this)(2, 2) = cy;
}

template <typename Real>
inline void ToraMatrix33<Real>::Set_A_Cardano(const ToraVector3<Real>& angles) {
  Real cx = std::cos(angles.x());
  Real cy = std::cos(angles.y());
  Real cz = std::cos(angles.z());
  Real sx = std::sin(angles.x());
  Real sy = std::sin(angles.y());
  Real sz = std::sin(angles.z());

  (*this)(0, 0) = (cx * cz) - (sz * sx * sy);
  (*this)(0, 1) = -sx * cy;
  (*this)(0, 2) = (cx * sz) + (sx * sy * cz);
  (*this)(1, 0) = (sx * cz) + (cx * sy * sz);
  (*this)(1, 1) = cy * cx;
  (*this)(1, 2) = (sx * sz) - (cx * sy * cz);
  (*this)(2, 0) = -sz * cy;
  (*this)(2, 1) = sy;
  (*this)(2, 2) = cy * cz;
}

template <typename Real>
inline void ToraMatrix33<Real>::Set_A_axis(const ToraVector3<Real>& X, const ToraVector3<Real>& Y,
                                           const ToraVector3<Real>& Z) {
  (*this)(0, 0) = X.x();
  (*this)(0, 1) = Y.x();
  (*this)(0, 2) = Z.x();
  (*this)(1, 0) = X.y();
  (*this)(1, 1) = Y.y();
  (*this)(1, 2) = Z.y();
  (*this)(2, 0) = X.z();
  (*this)(2, 1) = Y.z();
  (*this)(2, 2) = Z.z();
}

template <typename Real>
inline void ToraMatrix33<Real>::Set_A_Xdir(const ToraVector3<Real>& Xdir, const ToraVector3<Real>& Vsingular) {
  ToraVector3<Real> mX;
  ToraVector3<Real> mY;
  ToraVector3<Real> mZ;
  Xdir.DirToDxDyDz(mX, mY, mZ, Vsingular);
  this->Set_A_axis(mX, mY, mZ);
}

template <typename Real>
inline ToraVector3<Real> ToraMatrix33<Real>::Get_A_Eulero() const {
  ToraVector3<Real> eul;

  eul.y() = std::acos((*this)(2, 2));                       // rho, nutation
  eul.z() = std::acos((*this)(2, 1) / std::sin(eul.y()));   // csi, spin
  eul.x() = std::acos(-(*this)(1, 2) / std::sin(eul.y()));  // rho, nutation

  if (eul.y() == 0) {  // handle undefined initial position set
    eul.x() = 0;
    eul.z() = 0;
  }

  return eul;
}

template <typename Real>
inline ToraQuaternion<Real> ToraMatrix33<Real>::Get_A_quaternion() const {
  ToraQuaternion<Real> q;
  Real s, tr;
  Real half = (Real)0.5;

  Real m00 = (*this)(0, 0);
  Real m01 = (*this)(0, 1);
  Real m02 = (*this)(0, 2);
  Real m10 = (*this)(1, 0);
  Real m11 = (*this)(1, 1);
  Real m12 = (*this)(1, 2);
  Real m20 = (*this)(2, 0);
  Real m21 = (*this)(2, 1);
  Real m22 = (*this)(2, 2);

  tr = m00 + m11 + m22;  // diag sum

  if (tr >= 0) {
    s = std::sqrt(tr + 1);
    q.e0() = half * s;
    s = half / s;
    q.e1() = (m21 - m12) * s;
    q.e2() = (m02 - m20) * s;
    q.e3() = (m10 - m01) * s;
  } else {
    int i = 0;

    if (m11 > m00) {
      i = 1;
      if (m22 > m11) i = 2;
    } else {
      if (m22 > m00) i = 2;
    }

    switch (i) {
      case 0:
        s = std::sqrt(m00 - m11 - m22 + 1);
        q.e1() = half * s;
        s = half / s;
        q.e2() = (m01 + m10) * s;
        q.e3() = (m20 + m02) * s;
        q.e0() = (m21 - m12) * s;
        break;
      case 1:
        s = std::sqrt(m11 - m22 - m00 + 1);
        q.e2() = half * s;
        s = half / s;
        q.e3() = (m12 + m21) * s;
        q.e1() = (m01 + m10) * s;
        q.e0() = (m02 - m20) * s;
        break;
      case 2:
        s = std::sqrt(m22 - m00 - m11 + 1);
        q.e3() = half * s;
        s = half / s;
        q.e1() = (m20 + m02) * s;
        q.e2() = (m12 + m21) * s;
        q.e0() = (m10 - m01) * s;
        break;
    }
  }

  return q;
}

template <typename Real>
inline ToraVector3<Real> ToraMatrix33<Real>::Get_A_Xaxis() const {
  ToraVector3<Real> X;
  X.x() = (*this)(0, 0);
  X.y() = (*this)(1, 0);
  X.z() = (*this)(2, 0);
  return X;
}

template <typename Real>
inline ToraVector3<Real> ToraMatrix33<Real>::Get_A_Yaxis() const {
  ToraVector3<Real> Y;
  Y.x() = (*this)(0, 1);
  Y.y() = (*this)(1, 1);
  Y.z() = (*this)(2, 1);
  return Y;
}

template <typename Real>
inline ToraVector3<Real> ToraMatrix33<Real>::Get_A_Zaxis() const {
  ToraVector3<Real> Z;
  Z.x() = (*this)(0, 2);
  Z.y() = (*this)(1, 2);
  Z.z() = (*this)(2, 2);
  return Z;
}

template <typename Real>
inline ToraVector3<Real> ToraMatrix33<Real>::GetAx() const {
  return ToraVector3<Real>(0.5 * ((*this)(2, 1) - (*this)(1, 2)),  //
                           0.5 * ((*this)(0, 2) - (*this)(2, 0)),  //
                           0.5 * ((*this)(1, 0) - (*this)(0, 1)));
}

}  // namespace Tora