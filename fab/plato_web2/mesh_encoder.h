#ifndef _FAB_PLATO_WEB2_MESH_ENCODER_H__
#define _FAB_PLATO_WEB2_MESH_ENCODER_H__

#include <string>
#include <vector>
using std::string;
using std::vector;

#include <Eigen/Dense>

#include <boost/smart_ptr/shared_ptr.hpp>

#include "fab/geometry/tri_mesh.h"
#include "fab/geometry/template/shape.h"

namespace mit_plato {
namespace web {

// Output format is binary:
// 32-bit int for NUM_MESHES
// for each mesh:
// 2 32-bit integers [N_VERTICES N_TRIANGLES]
// 16 32-bit floats for MATRIX_TRANSFORM (row major order)
// followed by vertices and triangles
class MeshEncoder {
 public:
  static void EncodeAsString(const TriMesh& mesh, string* s);

  static void EncodeAsString(
      const vector<boost::shared_ptr<const mit_plato::Shape> >& shapes,
      string*s);

 private:
  static void Encode(size_t start_pos,
                     string* s,
                     int* metadata,
                     size_t meta_size,
                     float* matrix,
                     size_t mat_size,
                     float* positions,
                     size_t pos_size,
                     short int* triangles = NULL,
                     size_t tri_size = 0);

  static size_t SizeOfMeshEncoding(const TriMesh& mesh);

  static void ToArray(const Eigen::Matrix4d& transform, float* arr);

  static void EncodeAsString(const TriMesh& mesh,
                             const Eigen::Matrix4d& transform,
                             size_t start,
                             string* s);

  static void EncodeLarge(const TriMesh& mesh,
                          const Eigen::Matrix4d& transform,
                          size_t start_pos,
                          string* s);

};

}  // namespace web
}  // namespace mit_plato

#endif  // _FAB_PLATO_WEB2_MESH_ENCODER_H__
