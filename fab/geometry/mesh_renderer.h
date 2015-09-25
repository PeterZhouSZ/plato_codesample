#ifndef _FAB_GEOMETRY_TEMPLATE_RENDERER_H__
#define _FAB_GEOMETRY_TEMPLATE_RENDERER_H__

#include "fab/geometry/tri_mesh.h"

namespace mds {

// Knows how to render Mesh
// Has access to global rendering settings
// Provides utilities that various handles can use
class MeshRenderer {
 public:
  enum ShadingMode { WIREFRAME, FLAT, SMOOTH };

  struct Color {
    Color(int r, int g, int b, int a = 250) :
        red(r), green(g), blue(b), alpha(a) {}
    int red;
    int green;
    int blue;
    int alpha;
  };

  struct Options {
    Options() : shading_mode(SMOOTH),
                always_render_wireframe(true),
                render_selection(true),
                mesh_color_pointer(0),
                color(170, 170, 170, 250),
                overlay(false) {}

    ShadingMode shading_mode;
    bool always_render_wireframe;
    bool render_selection;
    float** mesh_color_pointer;  // overrides color
    Color color;  // default color
    bool overlay;  // if to render as overlay over orig. mesh
  };

  static void drawMesh(const TriMesh& mesh,
                       const Options& opts);

  static void drawSelectedVertices(const TriMesh& mesh);
  static void drawSelectedEdges(const TriMesh& mesh);
  static void drawSelectedFaces(const TriMesh& mesh);

  // TODO: add a way to set name offset, as more than
  // one mesh could potentially be drawn
  static void drawVerticesWithNames(const TriMesh& mesh);
  static void drawFacesWithNames(const TriMesh& mesh);
  static void drawEdgesWithNames(const TriMesh& mesh);

 private:
  static void drawMeshGeometry(
      const TriMesh& mesh,
      float** color_ptr = NULL);

  static void setShading(const ShadingMode mode);

  typedef TriMesh::Point Point;
};

}  // namespace mds

#endif  // _FAB_GEOMETRY_TEMPLATE_RENDERER_H__
