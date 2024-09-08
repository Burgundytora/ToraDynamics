// =============================================================================
// Authors: Tora
// =============================================================================

#include "Eigen/Dense"
#include "Eigen/Sparse"

namespace Tora {}  // namespace Tora

// Define Eigen::eigen_assert_exception. Note that this must precede inclusion of Eigen headers.
// Required for Eigen 3.3.8 (bug in Eigen?)
namespace Eigen {
static const bool should_raise_an_assert = false;

// Used to avoid to raise two exceptions at a time in which case the exception is not properly caught.
// This may happen when a second exceptions is triggered in a destructor.
static bool no_more_assert = false;
////static bool report_on_cerr_on_assert_failure = true;

struct eigen_assert_exception {
  eigen_assert_exception(void) {}
  ~eigen_assert_exception() { Eigen::no_more_assert = false; }
};

struct eigen_static_assert_exception {
  eigen_static_assert_exception(void) {}
  ~eigen_static_assert_exception() { Eigen::no_more_assert = false; }
};
}  // end namespace Eigen

namespace Tora {

/// Dense matrix with *dynamic size* (i.e., with unknown at compile time) and row-major storage.
/// A MatrixDynamic is templated by the coefficient type (default: double).
template <typename T = double>
using ToraMatrixDynamic = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// Dense matrix with *fixed size* (known at compile time) and row-major storage.
/// A MatrixNM is templated by the coefficient type and by the matrix dimensions (number of rows and columns).
template <typename T, int M, int N>
using ToraMatrixNM = Eigen::Matrix<T, M, N, Eigen::RowMajor>;

/// Dense matrix with *dynamic size* (i.e., with unknown at compile time) and column-major storage.
/// A MatrixDynamic_col is templated by the coefficient type (default: double).
template <typename T = double>
using ToraMatrixDynamic_col = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

/// Dense matrix with *fixed size* (known at compile time) and column-major storage.
/// A MatrixNM_col is templated by the coefficient type and by the matrix dimensions (number of rows and columns).
template <typename T, int M, int N>
using ToraMatrixNM_col = Eigen::Matrix<T, M, N, Eigen::ColMajor>;

////template <typename T, int M, int N>
////using MatrixNMnoalign = Eigen::Matrix<T, M, N, Eigen::RowMajor | Eigen::DontAlign>;

// -----------------------------------------------------------------------------

/// Column vector with *dynamic size* (i.e., with size unknown at compile time).
/// A VectorDynamic is templated by the type of its coefficients (default: double).
template <typename T = double>
using ToraVectorDynamic = Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

/// Row vector with *dynamic size* (i.e., with size unknown at compile time).
/// A RowVectorDynamic is templated by the type of its coefficients (default: double).
template <typename T = double>
using ToraRowVectorDynamic = Eigen::Matrix<T, 1, Eigen::Dynamic, Eigen::RowMajor>;

/// Column vector with *fixed size* (known at compile time).
/// A VectorN is templated by the type of its coefficients and the number of elements.
template <typename T, int N>
using ToraVectorN = Eigen::Matrix<T, N, 1>;

/// Row vector with *fixed size* (known at compile time).
/// A RowVectorN is templated by the type of its coefficients and the number of elements.
template <typename T, int N>
using ToraRowVectorN = Eigen::Matrix<T, 1, N, Eigen::RowMajor>;

// -----------------------------------------------------------------------------

/// General-purpose column array with *dynamic size*.
/// This class provides easy-access to coefficient-wise operations.
template <typename T = double>
using ToraArray = Eigen::Array<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

/// Column array with *fixed size* (known at compile time).
/// A ArrayN is templated by the type of its coefficients and the number of elements.
template <typename T, int N>
using ToraArrayN = Eigen::Array<T, N, 1>;

// -----------------------------------------------------------------------------

//// RADU
//// Consider templating the following by precision

/// Reference to a dense matrix expression, with double coefficients.
/// This allows writing non-template functions that can accept either a MatrixDynamic or a MatrixNM.
using ToraMatrixRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

/// Constant reference to a dense matrix expression, with double coefficients.
/// This allows writing non-template functions that can accept either a MatrixDynamic or a MatrixNM.
using ToraMatrixConstRef =
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>&;

/// Reference to a column vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a VectorDynamic or a VectorN.
using ToraVectorRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>;

/// Constant reference to a column vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a VectorDynamic or a RowVectorN.
using ToraVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>&;

/// Reference to a row vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a RowVectorDynamic or a CVectorN.
using ToraRowVectorRef = Eigen::Ref<Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>;

/// Constant reference to a row vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a RowVectorDynamic or a CVectorN.
using ToraRowVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>&;

/// Reference to an array expression, templated by coefficient type.
/// This allows writing non-template functions that can accept either a ArrayDynamic or a ArrayN.
template <typename T = double>
using ToraArrayRef = Eigen::Ref<Eigen::Array<T, Eigen::Dynamic, Eigen::ColMajor>>&;

/// Constant reference to an array expression, templated by coefficient type.
/// This allows writing non-template functions that can accept either a Array or a ArrayN.
template <typename T = double>
using ToraArrayConstRef = const Eigen::Ref<const Eigen::Array<T, Eigen::Dynamic, 1, Eigen::ColMajor>>&;

// -----------------------------------------------------------------------------

/// Sparse matrix representation.
/// A SparseMatrix is an Eigen SparseMatrix with double coefficients, row-major storage order, and int indices.
using ToraSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;

/// Utility function for slicing a vector based on an array of indices.
/// Return a new vector which only contains the elements with specified indices.
template <typename T = double>
ToraVectorDynamic<T> SliceVector(ToraVectorConstRef v, ToraArrayConstRef<int> indices) {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
  return v(indices);
#else
  return indices.unaryExpr(v);
#endif
}

}  // namespace Tora