#ifndef _FAB_GEOMETRY_TEMPLATE_PROTO_CONVERT_H__
#define _FAB_GEOMETRY_TEMPLATE_PROTO_CONVERT_H__

#include <Eigen/Dense>
#include "fab/geometry/template/operations.pb.h"

// Utils for converting between proto classes and regular classes

namespace mit_plato {

inline Eigen::Vector3d ToVector3d(const mit_plato::VectorParam& p) {
  Eigen::Vector3d res;
  res << p.x(), p.y(), p.z();
  return res;
}

inline mit_plato::VectorParam ToVectorParam(const Eigen::Vector3d& p) {
  mit_plato::VectorParam res;
  res.set_x(p.x());
  res.set_y(p.y());
  res.set_z(p.z());
  return res;
}

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_PROTO_CONVERT_H__
