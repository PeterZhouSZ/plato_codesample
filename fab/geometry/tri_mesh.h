#ifndef _FAB_GEOMETRY_TRI_MESH_H__
#define _FAB_GEOMETRY_TRI_MESH_H__

#include <set>
#include <vector>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
using std::set;
using std::vector;

// TODO: add a namespace

struct VertexElement_ {
  VertexElement_(int matr_ixd, int vert_idx)
      : matrix_idx(matr_ixd), vertex_idx(vert_idx) {}
  int matrix_idx;
  int vertex_idx;
};
struct HasMatrixIdx_ {
  HasMatrixIdx_(int matr_idx) : matrix_idx(matr_idx) {}
  bool operator()(const VertexElement_& ve) {
    return matrix_idx == ve.matrix_idx;
  }
  const int matrix_idx;
};
struct HasVertexIdx_ {
  HasVertexIdx_(int vert_idx) : vertex_idx(vert_idx) {}
  bool operator()(const VertexElement_& ve) {
    return vertex_idx == ve.vertex_idx;
  }
  const int vertex_idx;
};

// See:
// http://openmesh.org/Documentation/OpenMesh-2.1.1-Documentation/mesh_type.html
struct TriMeshTraits : public OpenMesh::DefaultTraits {
  typedef OpenMesh::Vec3d Point;  // osg::Vec3f
  typedef OpenMesh::Vec3d Normal;

  // Add normal property to vertices and faces, and no other for now
  // Later on, you might want to add color properties for visulization
  VertexAttributes  ( OpenMesh::Attributes::Normal |
                      OpenMesh::Attributes::Status );
  FaceAttributes    ( OpenMesh::Attributes::Normal |
                      OpenMesh::Attributes::Status);
  HalfedgeAttributes ( OpenMesh::Attributes::PrevHalfedge |
                       OpenMesh::Attributes::Status );
  EdgeAttributes ( OpenMesh::Attributes::PrevHalfedge |
                   OpenMesh::Attributes::Status );

  // See:
  // http://openmesh.org/Documentation/OpenMesh-2.0-Documentation/tutorial_07.html
  FaceTraits {
   public:
    FaceT() : group_id(0) {}
    int group_id;
  };
};

// Triangular mesh type
class TriMesh : public OpenMesh::TriMesh_ArrayKernelT<TriMeshTraits> {
 public:
  // Constructors
  TriMesh(void);
  virtual ~TriMesh(void);
  static TriMesh* read(const char* filename, OpenMesh::IO::Options* opt = NULL);
  bool resetFromFile(const char* filename, OpenMesh::IO::Options* opt = NULL);

  static void Combine(const vector<const TriMesh*>& meshes, TriMesh* res);

  // Display utils
  void needBoundingBox() const;
  const Point& bounding_box_min() const { return bbox_min; }
  const Point& bounding_box_max() const { return bbox_max; }
  Point getSceneCenter() const { return (bbox_min + bbox_max) / 2.0; }
  double getSceneRadius() const { return (bbox_max-bbox_min).norm() / 2.0; }
  TriMesh::Point getFaceCenter(const TriMesh::FaceHandle& fhandle);

  // Mesh manipulation
  void subdivideSelection();
  void doLaplacianSmoothingOnSelection(float lambda);
  bool edgeFlip(unsigned int idx);
  bool edgeFlip(const EdgeHandle& eh);
  void edgeCollapse(unsigned int idx);

  //! Tries to add face, and reorders face_vhandles if fails, adds group id
  //! to the face.
  FaceHandle smarterAddFace(
      const vector<TriMesh::VertexHandle>& face_vhandles,
      int group_id = 0);
  TriMesh::FaceHandle add_face(const vector<TriMesh::VertexHandle>& vhandles);
  TriMesh::FaceHandle add_face(TriMesh::VertexHandle v1,
                               TriMesh::VertexHandle v2,
                               TriMesh::VertexHandle v3);

  bool isComplex(const HalfedgeHandle& halfedge) const;

  double averageEdgeLength() const;
  double maxEdgeLength() const;
  double minEdgeLength() const;
  TriMesh::Point averageVertexPosition() const;

  // Util functions
  Point computeFaceCenter(const FaceHandle& f_handle);
  void filterAdjacentFaces(vector<FaceHandle>* face_handles);

  // Selection functions
  template <class H>
  const set<unsigned int>& getSelected() const;

  template <class H>
  bool isSelected(const H& handle) {
    const set<unsigned int>& sel = getSelected<H>();
    return sel.find(handle.idx()) != sel.end();
  }

  template <class H>
  void setSelected(const H& handle, bool selected) {
    set<unsigned int>& sel = getMutableSelected<H>();
    int index = handle.idx();
    if (selected) {
      sel.insert(index);
    } else if (sel.find(index) != sel.end()) {
      sel.erase(index);
    }
  }

  template <class H>
  void expandSelection(const unsigned int radius) {
    if (radius == 0) return;
    set<H> new_selected;
    getSelectionNeighbors(new_selected);
    set<unsigned int>& sel = getMutableSelected<H>();
    for (typename set<H>::const_iterator vh = new_selected.begin();
         vh != new_selected.end(); ++vh)
      sel.insert(vh->idx());
    expandSelection<H>(radius - 1);
  }

  template <class H>
  void shrinkSelection(const unsigned int radius) {
    if (radius == 0) return;
    // Get all selected entities on the border
    set<unsigned int> border;
    set<unsigned int>& sel = getMutableSelected<H>();
    for (set<unsigned int>::const_iterator idx = sel.begin();
         idx != sel.end(); ++idx) {
      set<H> neighbors;
      addNeighbors(tpl_handle<H>(*idx), neighbors);
      bool is_on_border = false;
      for (typename set<H>::const_iterator vh = neighbors.begin();
           vh != neighbors.end(); ++vh) {
        if (!isSelected(*vh)) is_on_border = true;
      }
      if (is_on_border) border.insert(*idx);
    }

    for (set<unsigned int>::const_iterator b_idx = border.begin();
         b_idx != border.end(); ++b_idx) {
      setSelected(tpl_handle<H>(*b_idx), false);
    }
    shrinkSelection<H>(radius - 1);
  }

  template <class H>
  void selectAll();

  void deselectAll();

  // Copies all the data from m and adds it to the existing data
  // (Warning: this method is stupid, does not check for duplicate
  // vertices, etc.)
  void AddData(const TriMesh& m);

  template <class H>
  H tpl_handle(unsigned int idx);

  // Add direct neighbor handles of a given handle to the set
  void addNeighbors(const FaceHandle& f_handle,
                     set<FaceHandle>& neighbors);
  void addNeighbors(const FaceHandle& f_handle,
                     set<VertexHandle>& neighbors);
  void addNeighbors(const FaceHandle& f_handle,
                     set<EdgeHandle>& neighbors);
  void addNeighbors(const VertexHandle& v_handle,
                     set<FaceHandle>& neighbors);
  void addNeighbors(const VertexHandle& v_handle,
                     set<VertexHandle>& neighbors);
  void addNeighbors(const VertexHandle& v_handle,
                     set<EdgeHandle>& neighbors);
  void addNeighbors(const EdgeHandle& e_handle,
                     set<FaceHandle>& neighbors);
  void addNeighbors(const EdgeHandle& e_handle,
                     set<VertexHandle>& neighbors);
  void addNeighbors(const EdgeHandle& e_handle,
                     set<EdgeHandle>& neighbors);

 protected:
  void computeEdgeLengthStatistics() const;

  template <class H>
  set<unsigned int>& getMutableSelected();

  template <class H>
  void getSelectionNeighbors(set<H>& new_selected) {
    for (set<unsigned int>::const_iterator f = selected_faces_.begin();
         f != selected_faces_.end(); ++f)
      addNeighbors(face_handle(*f), new_selected);
    for (set<unsigned int>::const_iterator e = selected_edges_.begin();
         e != selected_edges_.end(); ++e)
      addNeighbors(edge_handle(*e), new_selected);
    for (set<unsigned int>::const_iterator v = selected_vertices_.begin();
         v != selected_vertices_.end(); ++v)
      addNeighbors(vertex_handle(*v), new_selected);
  }

 private:
  mutable Point bbox_min, bbox_max;

  mutable double average_edge_length_;
  mutable double max_edge_length_;
  mutable double min_edge_length_;

  set<unsigned int> selected_vertices_;
  set<unsigned int> selected_edges_;
  set<unsigned int> selected_faces_;
};

#endif // _FAB_GEOMETRY_TRI_MESH_H__
