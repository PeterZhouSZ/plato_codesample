#ifndef _FAB_GEOMETRY_TRANSFORM_UTILS_H__
#define _FAB_GEOMETRY_TRANSFORM_UTILS_H__

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include "fab/geometry/convert.h"

namespace mds {

using namespace Eigen;

inline Matrix4d TranslationMat(const Vector3d& v) {
  Matrix4d res = Matrix4d::Identity();
  res.block(0, 3, 3, 1) = v;
  return res;
}

inline Matrix4d ScaleMat(const double scale) {
  Matrix4d mat = Matrix4d::Identity()* scale;
  mat(3, 3) = 1;
  return mat;
}

inline Matrix4d RotationYAxisMat(const double sin_angle,
                                 const double cos_angle) {
  Matrix4d mat;
  mat <<
      cos_angle, 0, sin_angle, 0,
      0, 1, 0, 0,
      -sin_angle, 0, cos_angle, 0,
      0, 0, 0, 1;
  return mat;
}

inline Matrix4d RotationAroundAxis(const Vector3d& axis,
                                   const double degrees) {
  Matrix4d mat(Matrix4d::Identity());
  mat.block(0, 0, 3, 3) = Eigen::AngleAxisd(
      mds::ToRadians(degrees),
      axis.normalized()).matrix();
  return mat;
}

inline Vector3d ApplyHomTransform(const Matrix4d& m,
                                  const Vector3d& v) {
  Vector4d position = Vector4d::Ones();
  position.head(3) = v;

  position = m * position;
  position /= position[3];
  return position.head(3);
}

inline Vector3d FindOrthoMostSimilarTo(const Vector3d& axis,
                                       const Vector3d& prev) {
  Matrix2d mat;
  double rep_val = -1.0 * axis[0] * axis[1] / axis[2];
  mat <<
      (1 - axis[0] * axis[0] / axis[2]), rep_val, rep_val,
      (1 - axis[1] * axis[1] / axis[2]);
  Vector2d b;
  b <<
      prev[0] + prev[2] * axis[0],
      prev[1] + prev[2] * axis[1];

  Vector2d xy = mat.inverse() * b;
  double z = -1.0 * (axis[0] * xy[0] + axis[1] * xy[1]) / axis[2];

  return Vector3d(xy[0], xy[1], z);
}

}  // namespace mds

#endif  // _FAB_GEOMETRY_TRANSFORM_UTILS_H__
