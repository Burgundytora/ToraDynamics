#include "ToraToraVector3.h"

namespace Tora {

const ToraVector3<double> VNULL(0., 0., 0.);
const ToraVector3<double> VECT_X(1., 0., 0.);
const ToraVector3<double> VECT_Y(0., 1., 0.);
const ToraVector3<double> VECT_Z(0., 0., 1.);

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ToraVector3<Real>::ToraVector3() {
  m_data[0] = 0;
  m_data[1] = 0;
  m_data[2] = 0;
}

template <class Real>
inline ToraVector3<Real>::ToraVector3(Real a) {
  m_data[0] = a;
  m_data[1] = a;
  m_data[2] = a;
}

template <class Real>
inline ToraVector3<Real>::ToraVector3(Real x, Real y, Real z) {
  m_data[0] = x;
  m_data[1] = y;
  m_data[2] = z;
}

template <class Real>
inline ToraVector3<Real>::ToraVector3(const ToraVector3<Real>& other) {
  m_data[0] = other.m_data[0];
  m_data[1] = other.m_data[1];
  m_data[2] = other.m_data[2];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator=(const ToraVector3<Real>& other) {
  if (&other == this) return *this;
  m_data[0] = other.m_data[0];
  m_data[1] = other.m_data[1];
  m_data[2] = other.m_data[2];
  return *this;
}

template <class Real>
template <class RealB>
inline ToraVector3<Real>& ToraVector3<Real>::operator=(const ToraVector3<RealB>& other) {
  m_data[0] = static_cast<Real>(other.m_data[0]);
  m_data[1] = static_cast<Real>(other.m_data[1]);
  m_data[2] = static_cast<Real>(other.m_data[2]);
  return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator+() const {
  return *this;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator-() const {
  return ToraVector3<Real>(-m_data[0], -m_data[1], -m_data[2]);
}

// -----------------------------------------------------------------------------
// Arithmetic operations

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator+(const ToraVector3<Real>& other) const {
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] + other.m_data[0];
  v.m_data[1] = m_data[1] + other.m_data[1];
  v.m_data[2] = m_data[2] + other.m_data[2];

  return v;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator-(const ToraVector3<Real>& other) const {
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] - other.m_data[0];
  v.m_data[1] = m_data[1] - other.m_data[1];
  v.m_data[2] = m_data[2] - other.m_data[2];

  return v;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator*(const ToraVector3<Real>& other) const {
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] * other.m_data[0];
  v.m_data[1] = m_data[1] * other.m_data[1];
  v.m_data[2] = m_data[2] * other.m_data[2];

  return v;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator/(const ToraVector3<Real>& other) const {
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] / other.m_data[0];
  v.m_data[1] = m_data[1] / other.m_data[1];
  v.m_data[2] = m_data[2] / other.m_data[2];

  return v;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator*(Real s) const {
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] * s;
  v.m_data[1] = m_data[1] * s;
  v.m_data[2] = m_data[2] * s;

  return v;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::operator/(Real s) const {
  Real oos = 1 / s;
  ToraVector3<Real> v;

  v.m_data[0] = m_data[0] * oos;
  v.m_data[1] = m_data[1] * oos;
  v.m_data[2] = m_data[2] * oos;

  return v;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator+=(const ToraVector3<Real>& other) {
  m_data[0] += other.m_data[0];
  m_data[1] += other.m_data[1];
  m_data[2] += other.m_data[2];

  return *this;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator-=(const ToraVector3<Real>& other) {
  m_data[0] -= other.m_data[0];
  m_data[1] -= other.m_data[1];
  m_data[2] -= other.m_data[2];

  return *this;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator*=(const ToraVector3<Real>& other) {
  m_data[0] *= other.m_data[0];
  m_data[1] *= other.m_data[1];
  m_data[2] *= other.m_data[2];

  return *this;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator/=(const ToraVector3<Real>& other) {
  m_data[0] /= other.m_data[0];
  m_data[1] /= other.m_data[1];
  m_data[2] /= other.m_data[2];

  return *this;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator*=(Real s) {
  m_data[0] *= s;
  m_data[1] *= s;
  m_data[2] *= s;

  return *this;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator/=(Real s) {
  Real oos = 1 / s;

  m_data[0] *= oos;
  m_data[1] *= oos;
  m_data[2] *= oos;

  return *this;
}

// -----------------------------------------------------------------------------
// SET Functions

template <class Real>
inline void ToraVector3<Real>::Set(Real x, Real y, Real z) {
  m_data[0] = x;
  m_data[1] = y;
  m_data[2] = z;
}

template <class Real>
inline void ToraVector3<Real>::Set(const ToraVector3<Real>& v) {
  m_data[0] = v.m_data[0];
  m_data[1] = v.m_data[1];
  m_data[2] = v.m_data[2];
}

template <class Real>
inline void ToraVector3<Real>::Set(Real s) {
  m_data[0] = s;
  m_data[1] = s;
  m_data[2] = s;
}

// -----------------------------------------------------------------------------
// EQUAL Functions

template <class Real>
inline bool ToraVector3<Real>::Equals(const ToraVector3<Real>& other) const {
  return (other.m_data[0] == m_data[0]) && (other.m_data[1] == m_data[1]) && (other.m_data[2] == m_data[2]);
}

template <class Real>
inline bool ToraVector3<Real>::Equals(const ToraVector3<Real>& other, Real tol) const {
  return (std::abs(other.m_data[0] - m_data[0]) < tol) && (std::abs(other.m_data[1] - m_data[1]) < tol) &&
         (std::abs(other.m_data[2] - m_data[2]) < tol);
}

// FUNCTIONS

template <class Real>
inline void ToraVector3<Real>::Add(const ToraVector3<Real>& A, const ToraVector3<Real>& B) {
  m_data[0] = A.m_data[0] + B.m_data[0];
  m_data[1] = A.m_data[1] + B.m_data[1];
  m_data[2] = A.m_data[2] + B.m_data[2];
}

template <class Real>
inline void ToraVector3<Real>::Sub(const ToraVector3<Real>& A, const ToraVector3<Real>& B) {
  m_data[0] = A.m_data[0] - B.m_data[0];
  m_data[1] = A.m_data[1] - B.m_data[1];
  m_data[2] = A.m_data[2] - B.m_data[2];
}

template <class Real>
inline void ToraVector3<Real>::Mul(const ToraVector3<Real>& A, Real s) {
  m_data[0] = A.m_data[0] * s;
  m_data[1] = A.m_data[1] * s;
  m_data[2] = A.m_data[2] * s;
}

template <class Real>
inline void ToraVector3<Real>::Scale(Real s) {
  m_data[0] *= s;
  m_data[1] *= s;
  m_data[2] *= s;
}

template <class Real>
inline void ToraVector3<Real>::Cross(const ToraVector3<Real>& A, const ToraVector3<Real>& B) {
  m_data[0] = (A.m_data[1] * B.m_data[2]) - (A.m_data[2] * B.m_data[1]);
  m_data[1] = (A.m_data[2] * B.m_data[0]) - (A.m_data[0] * B.m_data[2]);
  m_data[2] = (A.m_data[0] * B.m_data[1]) - (A.m_data[1] * B.m_data[0]);
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::Cross(const ToraVector3<Real> other) const {
  ToraVector3<Real> v;
  v.Cross(*this, other);
  return v;
}

template <class Real>
inline Real ToraVector3<Real>::Dot(const ToraVector3<Real>& B) const {
  return (m_data[0] * B.m_data[0]) + (m_data[1] * B.m_data[1]) + (m_data[2] * B.m_data[2]);
}

template <class Real>
inline Real ToraVector3<Real>::Length() const {
  return sqrt(Length2());
}

template <class Real>
inline Real ToraVector3<Real>::Length2() const {
  return this->Dot(*this);
}

template <class Real>
inline Real ToraVector3<Real>::LengthInf() const {
  return std::max(std::max(std::abs(m_data[0]), std::abs(m_data[1])), std::abs(m_data[2]));
}

template <class Real>
inline bool ToraVector3<Real>::Normalize() {
  Real length = this->Length();
  if (length < std::numeric_limits<Real>::min()) {
    m_data[0] = 1;
    m_data[1] = 0;
    m_data[2] = 0;
    return false;
  }
  this->Scale(1 / length);
  return true;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::GetNormalized() const {
  ToraVector3<Real> v(*this);
  v.Normalize();
  return v;
}

template <class Real>
inline void ToraVector3<Real>::SetLength(Real s) {
  Normalize();
  Scale(s);
}

template <class Real>
inline void ToraVector3<Real>::DirToDxDyDz(ToraVector3<Real>& Vx, ToraVector3<Real>& Vy, ToraVector3<Real>& Vz,
                                           const ToraVector3<Real>& Vsingular) const {
  // set Vx.
  if (this->IsNull())
    Vx = ToraVector3<Real>(1, 0, 0);
  else
    Vx = this->GetNormalized();

  Vz.Cross(Vx, Vsingular);
  Real zlen = Vz.Length();

  // if near singularity, change the singularity reference vector.
  if (zlen < 0.0001) {
    ToraVector3<Real> mVsingular;

    if (std::abs(Vsingular.m_data[0]) < 0.9)
      mVsingular = ToraVector3<Real>(1, 0, 0);
    else if (std::abs(Vsingular.m_data[1]) < 0.9)
      mVsingular = ToraVector3<Real>(0, 1, 0);
    else if (std::abs(Vsingular.m_data[2]) < 0.9)
      mVsingular = ToraVector3<Real>(0, 0, 1);

    Vz.Cross(Vx, mVsingular);
    zlen = Vz.Length();  // now should be nonzero length.
  }

  // normalize Vz.
  Vz.Scale(1 / zlen);

  // compute Vy.
  Vy.Cross(Vz, Vx);
}

template <class Real>
inline int ToraVector3<Real>::GetMaxComponent() const {
  int idx = 0;
  Real max = std::abs(m_data[0]);
  if (std::abs(m_data[1]) > max) {
    idx = 1;
    max = m_data[1];
  }
  if (std::abs(m_data[2]) > max) {
    idx = 2;
    max = m_data[2];
  }
  return idx;
}

template <class Real>
inline ToraVector3<Real> ToraVector3<Real>::GetOrthogonalVector() const {
  int idx1 = this->GetMaxComponent();
  int idx2 = (idx1 + 1) % 3;  // cycle to the next component
  int idx3 = (idx2 + 1) % 3;  // cycle to the next component

  // Construct v2 by rotating in the plane containing the maximum component
  ToraVector3<Real> v2(-m_data[idx2], m_data[idx1], m_data[idx3]);

  // Construct the normal vector
  ToraVector3<Real> ortho = Cross(v2);
  ortho.Normalize();
  return ortho;
}

}  // namespace Tora