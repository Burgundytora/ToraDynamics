// =============================================================================
// Authors: Tora
// =============================================================================

#ifndef TORA_ToraVector3_H
#define TORA_ToraVector3_H

namespace Tora {
template <class Real = double>
class ToraVector3 {
 private:
  Real m_data[3];

  /// Declaration of friend classes
  template <typename RealB>
  friend class Vector3;

 public:
  // CONSTRUCTORS
  ToraVector3();
  ToraVector3(Real x, Real y, Real z);
  ToraVector3(Real a);
  ToraVector3(const ToraVector3<Real>& other);

  /// Copy constructor with type change.
  template <class RealB>
  Vector3(const Vector3<RealB>& other);

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
  Vector3(const Eigen::MatrixBase<Derived>& vec,
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
  Vector3<Real> GetNormalized() const;

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

  ~ToraVector3();
};

}  // namespace Tora

#endif