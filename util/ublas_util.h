#ifndef _UTIL_UBLAS_UTIL__
#define _UTIL_UBLAS_UTIL__

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

namespace mds {

namespace ublas = boost::numeric::ublas;
using std::string;

bool ReadDmatMatrixFromFile(const string& filename,
                            ublas::matrix<double>* mat);

template <class T>
void ReadMatrixFromFile(const string& filename,
                        ublas::matrix<T>* mat) {
  std::ifstream ifs(filename.c_str());
  ifs >> *mat;
}

// Note: also need a special handling for uchar, but we
// don't need it here.
template <class T>
ublas::matrix<T> ReadMatrixFromString(const string& s) {
  std::stringstream ss(s, std::stringstream::in);
  ublas::matrix<T> mat;
  ss >> mat;
  return mat;
}

template <class T>
void WriteMatrixToFile(const string& filename,
                       const ublas::matrix<T>& mat) {
  std::ofstream ofs(filename.c_str());
  ofs << mat << std::endl;
}

// Matrix inversion routine.
// Uses lu_factorize and lu_substitute in uBLAS to invert a matrix;
// is taken from:
// http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?LU_Matrix_Inversion
template<class T>
bool InvertMatrix(const ublas::matrix<T>& input,
                  ublas::matrix<T>& inverse) {
  using namespace boost::numeric::ublas;
  typedef permutation_matrix<std::size_t> pmatrix;
  // create a working copy of the input
  ublas::matrix<T> A(input);
  // create a permutation matrix for the LU-factorization
  pmatrix pm(A.size1());
  // perform LU-factorization
  int res = lu_factorize(A,pm);
  if( res != 0 ) return false;
  // create identity matrix of "inverse"
  inverse.assign(ublas::identity_matrix<T>(A.size1()));
  // backsubstitute to get the inverse
  lu_substitute(A, pm, inverse);
  return true;
}

}  // namespace mds

#endif  // _UTIL_UBLAS_UTIL__
