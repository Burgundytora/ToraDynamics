// =============================================================================
// Authors: Tora
// =============================================================================

#ifndef TORA_ToraVector3_H
#define TORA_ToraVector3_H

#include "Eigen/Dense"
#include "Eigen/Sparse"

namespace Tora {

template <class Real = double>
class ToraVector3 {
 private:
  Real m_data[3];

  /// Declaration of friend classes
  template <typename RealB>
  friend class ToraVector3;

 public:
  // CONSTRUCTORS
  ToraVector3();
  ToraVector3(Real x, Real y, Real z);
  ToraVector3(Real a);
  ToraVector3(const ToraVector3<Real>& other);

  /// Copy constructor with type change.
  template <class RealB>
  ToraVector3(const ToraVector3<RealB>& other);

  /// Access to components
  Real& x() { return m_data[0]; }
  Real& y() { return m_data[1]; }
  Real& z() { return m_data[2]; }
  const Real& x() const { return m_data[0]; }
  const Real& y() const { return m_data[1]; }
  const Real& z() const { return m_data[2]; }

  /// Return const pointer to underlying array storage.
  const Real* data() const { return m_data; }

  // EIGEN INTER-OPERABILITY
  /// Construct a 3d vector from an Eigen vector expression.
  template <typename Derived>
  ToraVector3(const Eigen::MatrixBase<Derived>& vec,
              typename std::enable_if<(Derived::MaxRowsAtCompileTime == 1 || Derived::MaxColsAtCompileTime == 1),
                                      Derived>::type* = 0) {
    m_data[0] = vec(0);
    m_data[1] = vec(1);
    m_data[2] = vec(2);
  }

  // SET FUNCTIONS

  /// Set the three values of the vector at once.
  void Set(Real x, Real y, Real z);

  /// Set the vector as a copy of another vector.
  void Set(const ToraVector3<Real>& v);

  // EQUAL FUNCTIONS

  /// Return true if this vector is equal to another vector.
  bool Equals(const ToraVector3<Real>& other) const;

  /// Return true if this vector is equal to another vector, within a tolerance 'tol'.
  bool Equals(const ToraVector3<Real>& other, Real tol) const;

  // VECTOR NORMS

  /// Compute the euclidean norm of the vector, that is its length or magnitude.
  Real Length() const;

  /// Compute the squared euclidean norm of the vector.
  Real Length2() const;

  // OPERATORS OVERLOADING
  //
  // Note: c++ automatically creates temporary objects to store intermediate
  // results in long formulas, such as a= b*c*d, so the usage of operators
  // may give slower results than a wise (less readable however) usage of
  // Dot(), Cross() etc.. Also pay attention to C++ operator precedence rules!
  /// Subscript operator.

  //   [] need use assert, not good for perfomance.
  //   Real& operator[](unsigned index);
  //   const Real& operator[](unsigned index) const;

  /// Assignment operator (copy from another vector).
  ToraVector3<Real>& operator=(const ToraVector3<Real>& other);

  /// Assignment operator (copy from another vector) with type change.
  template <class RealB>
  ToraVector3<Real>& operator=(const ToraVector3<RealB>& other);

  /// Operators for sign change.
  ToraVector3<Real> operator+() const;
  ToraVector3<Real> operator-() const;

  /// Operator for vector sum.
  ToraVector3<Real> operator+(const ToraVector3<Real>& other) const;
  ToraVector3<Real>& operator+=(const ToraVector3<Real>& other);

  /// Operator for vector difference.
  ToraVector3<Real> operator-(const ToraVector3<Real>& other) const;
  ToraVector3<Real>& operator-=(const ToraVector3<Real>& other);

  /// Operator for element-wise multiplication.
  /// Note that this is neither dot product nor cross product.
  ToraVector3<Real> operator*(const ToraVector3<Real>& other) const;
  ToraVector3<Real>& operator*=(const ToraVector3<Real>& other);

  /// Operator for element-wise division.
  /// Note that 3D vector algebra is a skew field, non-divisional algebra,
  /// so this division operation is just an element-by element division.
  ToraVector3<Real> operator/(const ToraVector3<Real>& other) const;
  ToraVector3<Real>& operator/=(const ToraVector3<Real>& other);

  /// Operator for scaling the vector by a scalar value, as V*s
  ToraVector3<Real> operator*(Real s) const;
  ToraVector3<Real>& operator*=(Real s);

  /// Operator for scaling the vector by inverse of a scalar value, as v/s
  ToraVector3<Real> operator/(Real v) const;
  ToraVector3<Real>& operator/=(Real v);

  /// Operator for dot product: A^B means the scalar dot-product A*B
  /// Note: pay attention to operator low precedence (see C++ precedence rules!)
  Real operator^(const ToraVector3<Real>& other) const;

  /// Operator for cross product: A%B means the vector cross-product AxB
  /// Note: pay attention to operator low precedence (see C++ precedence rules!)
  ToraVector3<Real> operator%(const ToraVector3<Real>& other) const;
  ToraVector3<Real>& operator%=(const ToraVector3<Real>& other);

  /// Component-wise comparison operators
  bool operator<=(const ToraVector3<Real>& other) const;
  bool operator>=(const ToraVector3<Real>& other) const;
  bool operator<(const ToraVector3<Real>& other) const;
  bool operator>(const ToraVector3<Real>& other) const;
  bool operator==(const ToraVector3<Real>& other) const;
  bool operator!=(const ToraVector3<Real>& other) const;

  // FUNCTIONS

  /// Set this vector to the sum of A and B: this = A + B
  void Add(const ToraVector3<Real>& A, const ToraVector3<Real>& B);

  /// Set this vector to the difference of A and B: this = A - B
  void Sub(const ToraVector3<Real>& A, const ToraVector3<Real>& B);

  /// Set this vector to the product of a vector A and scalar s: this = A * s
  void Mul(const ToraVector3<Real>& A, Real s);

  /// Scale this vector by a scalar: this *= s
  void Scale(Real s);

  /// Set this vector to the cross product of A and B: this = A x B
  void Cross(const ToraVector3<Real>& A, const ToraVector3<Real>& B);

  /// Return the cross product with another vector: result = this x other
  ToraVector3<Real> Cross(const ToraVector3<Real> other) const;

  /// Return the dot product with another vector: result = this ^ B
  Real Dot(const ToraVector3<Real>& B) const;

  /// Normalize this vector in place, so that its euclidean length is 1.
  /// Return false if the original vector had zero length (in which case the vector
  /// is set to [1,0,0]) and return true otherwise.
  bool Normalize();

  /// Return a normalized copy of this vector, with euclidean length = 1.
  /// Not to be confused with Normalize() which normalizes in place.
  ToraVector3<Real> GetNormalized() const;

  /// Impose a new length to the vector, keeping the direction unchanged.
  void SetLength(Real s);

  /// Use the Gram-Schmidt orthonormalization to find the three
  /// orthogonal vectors of a coordinate system whose X axis is this vector.
  /// Vsingular (optional) sets the normal to the plane on which Dz must lie.
  void DirToDxDyDz(ToraVector3<Real>& Vx, ToraVector3<Real>& Vy, ToraVector3<Real>& Vz,
                   const ToraVector3<Real>& Vsingular = ToraVector3<Real>(0, 1, 0)) const;

  /// Return the index of the largest component in absolute value.
  int GetMaxComponent() const;

  /// Return a unit vector orthogonal to this vector
  ToraVector3<Real> GetOrthogonalVector() const;
};

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

template <class Real>
template <class RealB>
inline ToraVector3<Real>::ToraVector3(const ToraVector3<RealB>& other) {
  m_data[0] = static_cast<Real>(other.m_data[0]);
  m_data[1] = static_cast<Real>(other.m_data[1]);
  m_data[2] = static_cast<Real>(other.m_data[2]);
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
inline Real ToraVector3<Real>::operator^(const ToraVector3<Real>& other) const {
  return this->Dot(other);
}

template <class Real>
ToraVector3<Real> ToraVector3<Real>::operator%(const ToraVector3<Real>& other) const {
  ToraVector3<Real> v;
  v.Cross(*this, other);
  return v;
}

template <class Real>
inline ToraVector3<Real>& ToraVector3<Real>::operator%=(const ToraVector3<Real>& other) {
  this->Cross(*this, other);
  return *this;
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

// -----------------------------------------------------------------------------
// CONSTANTS

const ToraVector3<double> VNULL(0., 0., 0.);
const ToraVector3<double> VECT_X(1., 0., 0.);
const ToraVector3<double> VECT_Y(0., 1., 0.);
const ToraVector3<double> VECT_Z(0., 0., 1.);

// -----------------------------------------------------------------------------
// STATIC VECTOR MATH OPERATIONS
//
// These functions are here for people which prefer to use static
// functions instead of ToraVector3 class' member functions.
// NOTE: sometimes a wise adoption of the following functions may
// give faster results rather than using overloaded operators +/-/* in
// the vector class.
// For best readability of our code, it is suggested not to use
// these functions - use the member functions or operators of
// the ToraVector3 class instead.

inline std::ostream& operator<<(std::ostream& out, const ToraVector3<Real>& other) {
  out << other.x() << " " << other.y() << " " << other.z() << " ";
  return out;
}

template <class RealA, class RealB>
RealA Vdot(const ToraVector3<RealA>& va, const ToraVector3<RealB>& vb) {
  return (RealA)((va.x() * vb.x()) + (va.y() * vb.y()) + (va.z() * vb.z()));
}

template <class RealA>
void Vset(ToraVector3<RealA>& v, RealA mx, RealA my, RealA mz) {
  v.x() = mx;
  v.y() = my;
  v.z() = mz;
}

template <class RealA, class RealB>
ToraVector3<RealA> Vadd(const ToraVector3<RealA>& va, const ToraVector3<RealB>& vb) {
  ToraVector3<RealA> result;
  result.x() = va.x() + vb.x();
  result.y() = va.y() + vb.y();
  result.z() = va.z() + vb.z();
  return result;
}

template <class RealA, class RealB>
ToraVector3<RealA> Vsub(const ToraVector3<RealA>& va, const ToraVector3<RealB>& vb) {
  ToraVector3<RealA> result;
  result.x() = va.x() - vb.x();
  result.y() = va.y() - vb.y();
  result.z() = va.z() - vb.z();
  return result;
}

template <class RealA, class RealB>
ToraVector3<RealA> Vcross(const ToraVector3<RealA>& va, const ToraVector3<RealB>& vb) {
  ToraVector3<RealA> result;
  result.x() = (va.y() * vb.z()) - (va.z() * vb.y());
  result.y() = (va.z() * vb.x()) - (va.x() * vb.z());
  result.z() = (va.x() * vb.y()) - (va.y() * vb.x());
  return result;
}

template <class RealA, class RealB>
ToraVector3<RealA> Vmul(const ToraVector3<RealA>& va, RealB fact) {
  ToraVector3<RealA> result;
  result.x() = va.x() * (RealA)fact;
  result.y() = va.y() * (RealA)fact;
  result.z() = va.z() * (RealA)fact;
  return result;
}

template <class RealA>
RealA Vlength(const ToraVector3<RealA>& va) {
  return (RealA)va.Length();
}

template <class RealA>
ToraVector3<RealA> Vnorm(const ToraVector3<RealA>& va) {
  ToraVector3<RealA> result(va);
  result.Normalize();
  return result;
}

template <class RealA, class RealB>
bool Vequal(const ToraVector3<RealA>& va, const ToraVector3<RealB>& vb) {
  return (va == vb);
}

template <class RealA>
bool Vnotnull(const ToraVector3<RealA>& va) {
  return (va.x() != 0 || va.y() != 0 || va.z() != 0);
}

template <class RealA>
ToraVector3<RealA> Vmin(const ToraVector3<RealA>& va, const ToraVector3<RealA>& vb) {
  ToraVector3<RealA> result;
  result.x() = std::min(va.x(), vb.x());
  result.y() = std::min(va.y(), vb.y());
  result.z() = std::min(va.z(), vb.z());
  return result;
}

template <class RealA>
ToraVector3<RealA> Vmax(const ToraVector3<RealA>& va, const ToraVector3<RealA>& vb) {
  ToraVector3<RealA> result;
  result.x() = std::max(va.x(), vb.x());
  result.y() = std::max(va.y(), vb.y());
  result.z() = std::max(va.z(), vb.z());
  return result;
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplane(const ToraVector3<RealA>& va) {
  return asin(Vdot(va, ToraVector3<RealA>(1, 0, 0)));
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplaneNorm(const ToraVector3<RealA>& va) {
  return acos(Vdot(va, ToraVector3<RealA>(1, 0, 0)));
}

// Gets the angle of the projection on the YZ plane respect to
// the Y vector, as spinning about X.
template <class RealA>
double VangleRX(const ToraVector3<RealA>& va) {
  Vector vproj;
  vproj.x() = 0;
  vproj.y() = va.y();
  vproj.z() = va.z();
  vproj = Vnorm(vproj);
  if (vproj.x() == 1) return 0;
  return acos(vproj.y());
}

// The reverse of the two previous functions, gets the vector
// given the angle above the normal to YZ plane and the angle
// of rotation on X
template <class RealA>
ToraVector3<RealA> VfromPolar(double norm_angle, double pol_angle) {
  ToraVector3<> res;
  double projlen;
  res.x() = cos(norm_angle);  // 1) rot 'norm.angle'about z
  res.y() = sin(norm_angle);
  res.z() = 0;
  projlen = res.y();
  res.y() = projlen * cos(pol_angle);
  res.z() = projlen * sin(pol_angle);
  return res;
}

}  // namespace Tora

#endif