#ifndef _FAB_GEOMETRY_TEMPLATE_SHAPE_H__
#define _FAB_GEOMETRY_TEMPLATE_SHAPE_H__

#include <vector>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <Eigen/Dense>
#include <openvdb/openvdb.h>
#include <carve/carve.hpp>
#include <carve/polyhedron_decl.hpp>
#include "fab/geometry/tri_mesh.h"

namespace mit_plato {

//! Abstraction of shape that handles conversions between
//! shape representations. Implements Copy on Write via the copy
//! constructor; keeps track of mutable accessor calls and
//! updates other representations on demand.
//! E.g. if shape.MutableMesh() is called, next time shape.Volume()
//! is called, it will cause the voxel grid to be recomputed.
class Shape {
 public:
  typedef openvdb::FloatGrid GridType;

  Shape();

  Shape(const TriMesh& mesh);

  //! Creates a full copy; use copy constructor whenever possible.
  //  Shape* Copy() const;

  static void PolyhedronToMesh(const carve::poly::Polyhedron& poly,
                               TriMesh* mesh);

  const carve::poly::Polyhedron& Polyhedron() const;

  //! Initially creates a shallow copy of Shape, fully
  //! implements Copy on Write for mesh and volume.
  Shape(const Shape& s);

  //!
  //! Resets the contents of shape with those of the other shape
  void Reset(const Shape& s);

  //! Creates a lazy collection of shapes.
  //! These are not combined into a single volume or
  //! mesh until requested.
  Shape(const std::vector<boost::shared_ptr<const Shape> >& parts);

  //! May cause a copy of the mesh to be created if the
  //! shape object is a shallow copy of another.
  TriMesh& MutableMesh();

  //! May cause mesh to be generated from volume, if
  //! it does not yet exist.
  const TriMesh& Mesh() const;

  //! Returns mesh without applying any of the added transforms
  //! to the vertices.
  const TriMesh& MeshNoTransforms() const;

  //! Guaranteed to return non-NULL value. May cause a copy
  //! of the CoW grid to be created or an update based on
  //! mesh to be run.
  openvdb::FloatGrid::Ptr MutableVolume();

  //! Guaranteed to return non-NULL value. May cause volume to be
  //! created from mesh, if it does not yet exist or an update to be
  //! run based on mesh.
  openvdb::FloatGrid::ConstPtr Volume() const;

  //! Returns a deep copy of the volume for destructive CSG operations
  openvdb::FloatGrid::Ptr VolumeCopy() const;

  void log_info() const;

  void AddTransform(const Eigen::Matrix4d& transform);

  bool has_transform() const;

  const Eigen::Matrix4d& transform() const;

 protected:
  void clear_transform() const;
  void ApplyTransformToGrid() const;
  void ApplyTransformToMesh() const;

  void CombineMeshParts() const;
  void CombineVolumeParts() const;

  void UpdateVolumeFromMesh(const TriMesh& mesh) const;
  void UpdateMeshFromVolume(const openvdb::FloatGrid::Ptr grid) const;

  mutable boost::shared_ptr<TriMesh> copied_mesh_;
  mutable openvdb::FloatGrid::Ptr copied_grid_;

  mutable boost::shared_ptr<TriMesh> mesh_;
  mutable openvdb::FloatGrid::Ptr grid_;

  mutable boost::scoped_ptr<carve::poly::Polyhedron> poly_;

  // Set to true if mutable mesh/volume accessors have been called
  // TODO: is there a better way?
  mutable bool dirty_mesh_;
  mutable bool dirty_volume_;

  mutable Eigen::Matrix4d transform_;

  mutable vector<boost::shared_ptr<const Shape> > parts_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_SHAPE_H__
