#ifndef _FAB_GEOMETRY_CONVERT_H__
#define _FAB_GEOMETRY_CONVERT_H__

#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Dense>
#include <openvdb/openvdb.h>

#include "fab/geometry/tri_mesh.h"

namespace mds {

// CONSTANTS -------------------------------------------------------------------
inline double Pi() {
  static const double val = std::acos(-1.0);
  return val;
}

inline Eigen::Vector3d AverageVectors(
    const std::vector<Eigen::Vector3d>& vecs) {
  if (vecs.empty()) return Eigen::Vector3d::Zero();

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& vec : vecs) {
    sum = sum + vec;
  }
  sum = sum / static_cast<double>(vecs.size());
  return sum;
}

// COMPARISON FUNCTIONS --------------------------------------------------------
inline bool ApproxEqual(const double& v1, const double& v2,
                        const double EPS = std::numeric_limits<double>::epsilon()) {
  return fabs(v1 - v2) < EPS;
}

inline bool ApproxEqual(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs,
                        const double EPS = 0.0001) {
  return fabs(lhs[0] - rhs[0]) < EPS &&
         fabs(lhs[1] - rhs[1]) < EPS &&
         fabs(lhs[2] - rhs[2]) < EPS;
}

// CONVERSION FUNCTIONS --------------------------------------------------------
inline double ToRadians(const double degrees) {
  return degrees / 180.0 * std::acos(-1.0);
}

inline double ToDegrees(const double radians) {
  return radians * 180 / std::acos(-1.0);
}

inline TriMesh::Point ToMeshPoint(const Eigen::Vector3d& pt) {
  return TriMesh::Point(pt[0], pt[1], pt[2]);
}

// Note: need a different name b/c the call is ambiguous otherwise
inline TriMesh::Point ToMeshPoint4d(const Eigen::Vector4d& pt) {
  return TriMesh::Point(pt[0] / pt[3], pt[1] / pt[3], pt[2] / pt[3]);
}

inline TriMesh::Point ToMeshPoint(const openvdb::Vec3s& pt) {
  return TriMesh::Point(pt.x(), pt.y(), pt.z());
}

inline Eigen::Vector4d ToVector4d(const TriMesh::Point& pt) {
  Eigen::Vector4d res;
  res << pt[0], pt[1], pt[2], 1.0;
  return res;
}

inline openvdb::Vec3s ToVdbVec3(const TriMesh::Point& pt) {
  return openvdb::Vec3s(pt[0], pt[1], pt[2]);
}

inline Eigen::Vector3d ToEigenVector3d(const TriMesh::Point& pt) {
  Eigen::Vector3d vec;
  vec << pt[0], pt[1], pt[2];
  return vec;
}

inline void ToGLMatrix(const Eigen::Matrix4d& mat, float transform[16]) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      transform[col * 4 + row] = mat(row, col);
    }
  }
}

// MISC ------------------------------------------------------------------------

inline double GetAngleXZ(const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
  Eigen::Vector2d u_prime;
  u_prime << u[0], u[2];
  Eigen::Vector2d v_prime;
  v_prime << v[0], v[2];

  if (ApproxEqual(u_prime.norm(), 0.0) ||
      ApproxEqual(v_prime.norm(), 0.0)) {
    return 0;
  }

  double cos_angle = u_prime.dot(v_prime) / u_prime.norm() / v_prime.norm();
  double theta = std::acos(cos_angle);

  Eigen::Matrix2d mat;
  mat.col(0) = u_prime;
  mat.col(1) = v_prime;
  double det = mat.determinant();
  if (ApproxEqual(det, 0.0)) {
    // TODO: determine if is -Pi
    // Vectors are parallel
    return 0.0;
  } else if (det > 0) {
    return theta;
  } else {
    return -theta;
  }
}

}  // mds

#endif  //_FAB_GEOMETRY_CONVERT_H__
