#ifndef _FAB_GEOMETRY_TRIANGLE_WRAP_H__
#define _FAB_GEOMETRY_TRIANGLE_WRAP_H__

extern "C" {
#include <triangle/triangle.h>
}

namespace mds {

void InitTriangulateIo(triangulateio* input);

}  // namespace mds

#endif  // _FAB_GEOMETRY_MESH_DEFORMER_H__
