#include <algorithm>
#include <map>
using std::map;

#include <vector>
using std::vector;

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <carve/csg_triangulator.hpp>
#include <carve/input.hpp>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>

#include "fab/geometry/template/shape.h"
#include "fab/geometry/convert.h"
#include "fab/geometry/transform_utils.h"

DEFINE_double(shape_volume_precision, 0.1, "Shape precision");

namespace mit_plato {

using Eigen::Vector3d;

namespace {

void LogMeshInfo(const std::string& prefix, const TriMesh* m) {
  if (!m) {
    VLOG(3) << prefix << " NULL";
  } else {
    VLOG(3) << prefix << m->n_vertices() << " vertices "
              << m->n_faces() << " faces";
  }
}

void LogGridInfo(const std::string& prefix, openvdb::FloatGrid::Ptr g) {
   if (!g) {
    VLOG(3) << prefix << " NULL";
  } else {
     VLOG(3) << prefix << g->activeVoxelCount() << " voxels";
   }
}

}  // namespace

Shape::Shape()
    : dirty_mesh_(false),
      dirty_volume_(false),
      transform_(Eigen::Matrix4d::Identity()) {}

Shape::Shape(const TriMesh& mesh)
    : dirty_mesh_(false),
      dirty_volume_(true),
      transform_(Eigen::Matrix4d::Identity()) {
  mesh_.reset(new TriMesh(mesh));
}

Shape::Shape(const std::vector<boost::shared_ptr<const Shape> >& parts)
    : dirty_mesh_(false),
      dirty_volume_(false),
      transform_(Eigen::Matrix4d::Identity()),
      parts_(parts) {}

Shape::Shape(const Shape& s) : dirty_mesh_(s.dirty_mesh_),
                               dirty_volume_(s.dirty_volume_),
                               transform_(s.transform_) {
  if (!dirty_mesh_) {
    if (s.mesh_) {
      copied_mesh_ = s.mesh_;
    } else if (s.copied_mesh_) {
      copied_mesh_ = s.copied_mesh_;
    }
  }

  if (!dirty_volume_) {
    if (s.grid_) {
      copied_grid_ = s.grid_;
    } else if (s.copied_grid_) {
      copied_grid_ = s.copied_grid_;
    }
  }
}

void Shape::PolyhedronToMesh(const carve::poly::Polyhedron& poly,
                             TriMesh* mesh) {
  mesh->clear();

  vector<TriMesh::VertexHandle> pt_handles;
  for (const auto& vert : poly.vertices) {
    TriMesh::Point p = TriMesh::Point(vert.v[0], vert.v[1], vert.v[2]);
    pt_handles.push_back(mesh->add_vertex(p));
  }

  for (size_t i = 0; i < poly.faces.size(); ++i) {
    // Actual type: carve::poly::Face<3> (CLANG bug causes it to crash)
    auto& f = poly.faces[i];
    std::vector<carve::triangulate::tri_idx> result;

    std::vector<const carve::poly::Polyhedron::vertex_t *> vloop;
    f.getVertexLoop(vloop);

    carve::triangulate::triangulate(carve::poly::p2_adapt_project<3>(f.project), vloop, result);
    carve::triangulate::improve(carve::poly::p2_adapt_project<3>(f.project), vloop, result);

    for (size_t j = 0; j < result.size(); ++j) {
      vector<TriMesh::VertexHandle> face_vhandles;
      face_vhandles.push_back(pt_handles.at(
          poly.vertexToIndex_fast(vloop[result[j].a])));
      face_vhandles.push_back(pt_handles.at(
          poly.vertexToIndex_fast(vloop[result[j].b])));
      face_vhandles.push_back(pt_handles.at(
          poly.vertexToIndex_fast(vloop[result[j].c])));

      if (!mesh->smarterAddFace(face_vhandles).is_valid()) {
        LOG(ERROR) << "Failed to add first triangle of a quad";
      }
    }
  }

  mesh->update_normals();
}


const carve::poly::Polyhedron& Shape::Polyhedron() const {
  if (poly_ != NULL) {
    return *poly_;
  }

  carve::input::PolyhedronData data;

  map<TriMesh::VertexHandle, int> handle_to_id;
  for (TriMesh::ConstVertexIter vit = Mesh().vertices_begin();
       vit != Mesh().vertices_end(); ++vit) {
    Vector3d pt = mds::ApplyHomTransform(
        transform_, mds::ToEigenVector3d(Mesh().point(vit)));
    handle_to_id[vit.handle()] = data.addVertex(carve::geom::VECTOR(pt[0], pt[1], pt[2]));
  }

  for (TriMesh::ConstFaceIter fit = Mesh().faces_begin();
       fit != Mesh().faces_end(); ++fit) {
    vector<int> verts;
    for (TriMesh::ConstFaceVertexIter vit = Mesh().cfv_iter(fit); vit; ++vit) {
      verts.push_back(handle_to_id.at(vit.handle()));
    }
    CHECK_EQ(3, verts.size());
    data.addFace(verts[0], verts[1], verts[2]);
  }
  poly_.reset(data.create());
  return *poly_;
}

void Shape::Reset(const Shape& s) {
  dirty_mesh_ = s.dirty_mesh_;
  dirty_volume_ = s.dirty_volume_;
  transform_ = s.transform_;

  if (s.mesh_) {
    copied_mesh_ = s.mesh_;
  } else if (s.copied_mesh_) {
    copied_mesh_ = s.copied_mesh_;
  }

  if (s.grid_) {
    copied_grid_ = s.grid_;
  } else if (s.copied_grid_) {
    copied_grid_ = s.copied_grid_;
  }
}

void Shape::log_info() const {
  LogMeshInfo("mesh_", mesh_.get());
  LogMeshInfo("copied_mesh_", copied_mesh_.get());

  LogGridInfo("grid_", grid_);
  LogGridInfo("copied_grid_", copied_grid_);

  for (int i = 0; i < parts_.size(); ++i) {
    VLOG(1) << "Part " << i << ":";
    parts_[i]->log_info();
  }
}

void Shape::AddTransform(const Eigen::Matrix4d& transform) {
  transform_ = transform * transform_;
}

void Shape::clear_transform() const {
  transform_ = Eigen::Matrix4d::Identity();
}

bool Shape::has_transform() const {
  return !transform_.isApprox(Eigen::Matrix4d::Identity());
}

const Eigen::Matrix4d& Shape::transform() const {
  return transform_;
}

void Shape::ApplyTransformToGrid() const {
  openvdb::math::Mat4d mat(
      transform_(0,0), transform_(0,1), transform_(0,2), transform_(0,3),
      transform_(1,0), transform_(1,1), transform_(1,2), transform_(1,3),
      transform_(2,0), transform_(2,1), transform_(2,2), transform_(2,3),
      transform_(3,0), transform_(3,1), transform_(3,2), transform_(3,3));
  openvdb::tools::GridTransformer transformer(mat);

  if (!grid_) {
    CHECK(copied_grid_);
    UpdateVolumeFromMesh(TriMesh());

    // Resample using nearest-neighbor interpolation.
    transformer.transformGrid<openvdb::tools::PointSampler, openvdb::FloatGrid>(
        *copied_grid_, *grid_);
    copied_grid_.reset();
  } else {
    openvdb::FloatGrid::Ptr prev_grid = grid_;
    UpdateVolumeFromMesh(TriMesh());

    transformer.transformGrid<openvdb::tools::PointSampler, openvdb::FloatGrid>(
        *prev_grid, *grid_);
  }
  grid_->tree().prune();
}

void Shape::ApplyTransformToMesh() const {
  if (mesh_ == NULL) {
    CHECK(copied_mesh_ != NULL);
    mesh_.reset(new TriMesh(*copied_mesh_));
    copied_mesh_.reset();
  }

  LOG(INFO) << "Before transform";
  Eigen::MatrixXd locations =  Eigen::MatrixXd::Ones(4, mesh_->n_vertices());
  int idx = 0;
  for (TriMesh::VertexIter vit = mesh_->vertices_begin();
       vit != mesh_->vertices_end(); ++vit) {
    const TriMesh::Point& location = mesh_->point(vit.handle());
    Eigen::MatrixXd::Index col = static_cast<Eigen::MatrixXd::Index>(vit.handle().idx());
    locations(0, idx) = location[0];
    locations(1, idx) = location[1];
    locations(2, idx) = location[2];
    ++idx;
  }
  locations = transform_ * locations;

  // TODO: Normalize by homogeneous coord if necessary!
  idx = 0;
  for (TriMesh::VertexIter vit = mesh_->vertices_begin();
       vit != mesh_->vertices_end(); ++vit) {
    Eigen::MatrixXd::Index col = static_cast<Eigen::MatrixXd::Index>(vit.handle().idx());
    mesh_->set_point(vit.handle(),
                     TriMesh::Point(
                         locations(0, idx),
                         locations(1, idx),
                         locations(2, idx)));
    ++idx;
  }
  LOG(INFO) << "After transform";
  mesh_->update_normals();
}

const TriMesh& Shape::Mesh() const {
  if (!parts_.empty()) {
    CombineMeshParts();
  }

  if (dirty_mesh_) {
    if (!mesh_) {
      mesh_.reset(new TriMesh());
      copied_mesh_.reset();
    }
    if (has_transform()) {
      ApplyTransformToGrid();
      clear_transform();
    }
    UpdateMeshFromVolume((grid_ ? grid_ : copied_grid_));
    return *mesh_;
  }

  if (!mesh_) {
    if (copied_mesh_) {
      if (!has_transform()) {
        return *copied_mesh_;
      } else {
        ApplyTransformToMesh();
        dirty_volume_ = true;
        clear_transform();
        return *mesh_;
      }
    }

    mesh_.reset(new TriMesh());
    if (grid_ || copied_grid_) {
      if (has_transform()) {
        ApplyTransformToGrid();
        clear_transform();
      }
      UpdateMeshFromVolume((grid_ ? grid_ : copied_grid_));
    }
  } else {
    if (has_transform()) {
      ApplyTransformToMesh();
      dirty_volume_ = true;
      clear_transform();
    }
  }

  return *mesh_;
}

const TriMesh& Shape::MeshNoTransforms() const {
  if (!parts_.empty()) {
    CombineMeshParts();
  }

  if (dirty_mesh_) {
    if (!mesh_) {
      mesh_.reset(new TriMesh());
      copied_mesh_.reset();
    }
    UpdateMeshFromVolume((grid_ ? grid_ : copied_grid_));
  }

  if (!mesh_) {
    if (copied_mesh_) {
      return *copied_mesh_;
    }

    mesh_.reset(new TriMesh());
    if (grid_ || copied_grid_) {
      UpdateMeshFromVolume((grid_ ? grid_ : copied_grid_));
    }
  }

  return *mesh_;
}

openvdb::FloatGrid::Ptr Shape::VolumeCopy() const {
  return Volume()->deepCopy();
}

openvdb::FloatGrid::ConstPtr Shape::Volume() const {
  if (!parts_.empty()) {
    CombineVolumeParts();
  }

  if (dirty_volume_) {
    if (!grid_) {
      copied_grid_.reset();
    }
    if (has_transform()) {
      ApplyTransformToMesh();
      clear_transform();
    }
    // Resets grid_
    UpdateVolumeFromMesh((mesh_ ? *mesh_ : *copied_mesh_));
  }

  if (!grid_) {
    if (copied_grid_) {
      if (!has_transform()) {
        return copied_grid_;
      } else {
        ApplyTransformToGrid();
        dirty_mesh_ = true;
        clear_transform();
        return grid_;
      }
    }

    if (mesh_ || copied_mesh_) {
      if (has_transform()) {
        ApplyTransformToMesh();
        clear_transform();
      }
      UpdateVolumeFromMesh((mesh_ ? *mesh_ : *copied_mesh_));  // Resets grid_
    } else {
      clear_transform();
      UpdateVolumeFromMesh(TriMesh());
    }
  } else {
    if (has_transform()) {
      ApplyTransformToGrid();
      dirty_mesh_ = true;
      clear_transform();
    }
  }

  return grid_;
}

TriMesh& Shape::MutableMesh() {
  poly_.reset(NULL);
  Mesh();  // runs any updates from grid_
  if (!mesh_) {
    CHECK(copied_mesh_)
        << "This is a bug; mesh_ can be NULL after a call to Mesh() only "
        << "if copied_mesh_ is set.";

    mesh_.reset(new TriMesh(*copied_mesh_));
    copied_mesh_.reset();
  }

  dirty_volume_ = true;
  return *mesh_;
}

openvdb::FloatGrid::Ptr Shape::MutableVolume() {
  poly_.reset(NULL);
  Volume();  // runs any updates from mesh_
  if (!grid_) {
    CHECK(copied_grid_)
        << "This is a bug; grid_ can be NULL after a call to Grid() only "
        << "if copied_grid_ is set.";

    grid_ = copied_grid_->deepCopy();
    copied_grid_.reset();
  }

  dirty_mesh_ = true;
  return grid_;
}

void Shape::UpdateVolumeFromMesh(const TriMesh& mesh) const {
  openvdb::initialize();

  map<TriMesh::VertexHandle, openvdb::Index32> handle_to_id;
  vector<openvdb::Vec3s> points;
  for (TriMesh::ConstVertexIter vit = mesh.vertices_begin();
       vit != mesh.vertices_end(); ++vit) {
    handle_to_id[vit.handle()] = static_cast<openvdb::Index32>(points.size());
    points.push_back(mds::ToVdbVec3(mesh.point(vit)));
  }

  vector<openvdb::Vec3I> triangles;
  for (TriMesh::ConstFaceIter fit = mesh.faces_begin();
       fit != mesh.faces_end(); ++fit) {
    vector<openvdb::Index32> verts;
    for (TriMesh::ConstFaceVertexIter vit = mesh.cfv_iter(fit); vit; ++vit) {
      verts.push_back(handle_to_id.at(vit.handle()));
    }
    CHECK_EQ(3, verts.size());
    triangles.push_back(openvdb::Vec3I(verts[0], verts[1], verts[2]));
  }

  VLOG(1) << "Creating grid from " << points.size() << " points, "
          << triangles.size() << " triangles";

  openvdb::math::Transform::Ptr p =
      openvdb::math::Transform::createLinearTransform(FLAGS_shape_volume_precision);
  vector<openvdb::Vec4I> quads;

  grid_ = //openvdb::tools::meshToUnsignedDistanceField<openvdb::FloatGrid>(
      openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
          *p, points, triangles, quads);

  dirty_volume_ = false;
  grid_->tree().prune();
  VLOG(1) << "Grid created OK";
}

void Shape::UpdateMeshFromVolume(const openvdb::FloatGrid::Ptr grid) const {
  // OpenVDB method
  std::vector<openvdb::Vec3s> points;
  std::vector<openvdb::Vec3I> triangles;
  std::vector<openvdb::Vec4I> quads;

  openvdb::tools::volumeToMesh(*grid, points, triangles, quads); //, 0.0, 5.0);

  VLOG(1) << "Created " << points.size() << " points, "
          << triangles.size() << " triangles, "
          << quads.size() << "quads";

  mesh_->clear();

  vector<TriMesh::VertexHandle> pt_handles;
  for (const openvdb::Vec3s& pt : points) {
    pt_handles.push_back(mesh_->add_vertex(mds::ToMeshPoint(pt)));
  }

  for (const openvdb::Vec3I& tri : triangles) {
    vector<TriMesh::VertexHandle> face_vhandles;
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(tri.x())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(tri.y())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(tri.z())));
    mesh_->smarterAddFace(face_vhandles);
  }

  for (const openvdb::Vec4I& quad : quads) {
    vector<TriMesh::VertexHandle> face_vhandles;
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.z())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.y())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.x())));
    if (!mesh_->smarterAddFace(face_vhandles).is_valid()) {
      LOG(ERROR) << "Failed to add first triangle of a quad";
    }
    face_vhandles.clear();
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.w())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.z())));
    face_vhandles.push_back(pt_handles.at(static_cast<unsigned>(quad.x())));
    if (!mesh_->smarterAddFace(face_vhandles).is_valid()) {
      LOG(ERROR) << "Failed to add 2nd triangle of a quad";
    }
  }
  mesh_->update_normals();
  dirty_mesh_ = false;
}

void Shape::CombineMeshParts() const {
  CHECK(mesh_ == NULL);
  CHECK(copied_mesh_ == NULL);
  CHECK(grid_ == NULL);
  CHECK(copied_grid_ == NULL);
  CHECK(!parts_.empty());

  mesh_.reset(new TriMesh());
  for (int i = 0; i < parts_.size(); ++i) {
    mesh_->AddData(parts_[i]->Mesh());
  }

  dirty_volume_ = true;
  parts_.clear();
}

void Shape::CombineVolumeParts() const {
  CHECK(mesh_ == NULL);
  CHECK(copied_mesh_ == NULL);
  CHECK(grid_ == NULL);
  CHECK(copied_grid_ == NULL);
  CHECK(!parts_.empty());

  UpdateVolumeFromMesh(TriMesh());
  for (int i = 0; i < parts_.size(); ++i) {
    openvdb::tools::csgUnion(
        *grid_,
        *parts_[i]->VolumeCopy());
  }

  dirty_mesh_ = true;
  parts_.clear();
}

}  // namespace mit_plato
