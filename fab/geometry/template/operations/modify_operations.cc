#include <cstdlib>

#include <Eigen/Geometry>

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <carve/polyhedron_decl.hpp>
#include <carve/csg.hpp>
#include <gflags/gflags.h>

#include "fab/geometry/convert.h"
#include "fab/geometry/transform_utils.h"
#include "fab/geometry/template/modify_operations.h"
#include "fab/geometry/template/proto_convert.h"
#include "fab/geometry/template/registration.h"


DEFINE_bool(debug_turn_into_tentacle, false,
              "If true skips proper CSG");

namespace mit_plato {

using namespace Eigen;

REGISTER_OPERATION_WITH_DEFAULT(
    "Scale", ScaleOperation, ScaleOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "Translate", TranslateOperation, TranslateOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "Rotate", RotateOperation, RotateOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "Perturb surface", RandomDisplacementOperation, RandomDisplacementOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "Laplacian smoothing", SmoothOperation, SmoothOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "Subdivide", SubdivideOperation, SubdivideOpSpec);
REGISTER_OPERATION_WITH_DEFAULT(
    "TurnIntoTentacle", TurnIntoTentacleOperation, TurnIntoTentacleOpSpec);

// SMOOTH ----------------------------------------------------------------------
void SmoothOperation::Modify(Shape* shape) {
  TriMesh& mesh = shape->MutableMesh();
  mesh.selectAll<TriMesh::VertexHandle>();
  mesh.doLaplacianSmoothingOnSelection(spec_.lambda());
  mesh.deselectAll();
  mesh.update_normals();
}

// SUBDIVIDE -------------------------------------------------------------------
void SubdivideOperation::Modify(Shape* shape) {
  TriMesh& mesh = shape->MutableMesh();
  for (int i = 0; i < spec_.n_times(); ++i) {
    mesh.selectAll<TriMesh::FaceHandle>();
    mesh.subdivideSelection();
  }
  mesh.deselectAll();
  mesh.update_normals();
}

// RANDOM DISPLACEMENT ---------------------------------------------------------
void RandomDisplacementOperation::Modify(Shape* shape) {
  TriMesh& mesh = shape->MutableMesh();
  for (TriMesh::VertexIter vit = mesh.vertices_begin();
       vit != mesh.vertices_end(); ++vit) {
    TriMesh::Point location = mesh.point(vit.handle());
    TriMesh::Point normal = mesh.normal(vit.handle());

    double amount =
        (static_cast<double>(rand()) / (RAND_MAX + 1) - 0.5) *
        spec_.amount();

    mesh.set_point(vit.handle(), location + normal * amount);
  }
  mesh.update_normals();
}

// TRANSFORM -------------------------------------------------------------------
void TransformOperation::Modify(Shape* shape) {
  if (spec_.hom_matrix_size() == 0) return;

  Eigen::Matrix4d mat;
  CHECK_EQ(16, spec_.hom_matrix_size());
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      mat(row, col) = spec_.hom_matrix(row * 4 + col);
    }
  }
  VLOG(3) << "Read matrix: " << mat;
  shape->AddTransform(mat);
}

void TransformOperation::ToSpec(
    const Matrix4d& mat, TransformOpSpec* spec) const {
  spec->clear_hom_matrix();
  for (int i = 0; i < 16; ++i) {
    spec->add_hom_matrix(0);
  }

  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      spec->set_hom_matrix(row * 4 + col, mat(row, col));
    }
  }
}

// SCALE -----------------------------------------------------------------------
void ScaleOperation::Modify(Shape* shape) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity() * spec_.scale();
  mat(3, 3) = 1;
  shape->AddTransform(mat);
}

// TRANSLATE -------------------------------------------------------------------
TranslateOperation::TranslateOperation(const TranslateOpSpec& spec) {
  spec_.CopyFrom(spec);
  ExposeParams(&spec_);
  Reset();
}

void TranslateOperation::Reset() {
  mat_ <<
      1, 0, 0, spec_.translation().x(),
      0, 1, 0, spec_.translation().y(),
      0, 0, 1, spec_.translation().z(),
      0, 0, 0, 1;
}

void TranslateOperation::Modify(Shape* shape) {
  Reset();
  shape->AddTransform(mat_);
}

// ROTATE ----------------------------------------------------------------------
RotateOperation::RotateOperation(const RotateOpSpec& spec) {
  spec_.CopyFrom(spec);
  ExposeParams(&spec_);
  Reset();
}

void RotateOperation::Reset() {
  mat_ = Matrix4d::Identity();

  Eigen::Vector3d axis = ToVector3d(spec_.axis());
  if (!axis.isZero()) {
    mat_.block(0, 0, 3, 3) = Eigen::AngleAxisd(
        mds::ToRadians(spec_.degrees()),
        axis.normalized()).matrix();
  }
}

void RotateOperation::Modify(Shape* shape) {
  Reset();
  shape->AddTransform(mat_);
}

// TURN INTO TENTACLE ----------------------------------------------------------

Eigen::Matrix4d TurnIntoTentacleOperation::one_link(double A,
                double B,
                double ampl1,
                double ampl2,
                double sphere_radius,
                double t) const {
  double rad = spec_.base_radius() + spec_.a() * sqrt(t) +
               ampl1 * sin(spec_.horizontal_period() * t);
  double h = B * t * t +
             ampl2 * std::sin(spec_.vertical_period() * t);
  double theta = t;

  return mds::RotationYAxisMat(std::sin(theta), std::cos(theta)) *
      mds::TranslationMat(Vector3d(rad, h, 0)) *
      mds::ScaleMat(sphere_radius * base_scale_);
}

void TurnIntoTentacleOperation::add_transform_recursive(
    double max_t,
    double A,
    double B,
    double ampl1,
    double ampl2,
    double alpha,
    double beta,
    double t,
    double it_num,
    vector<Eigen::Matrix4d>* transforms) const {
  double sphere_radius = alpha * exp(-beta * t);
  transforms->push_back(one_link(A, B, ampl1, ampl2, sphere_radius, t));

  double mult1 = spec_.horizontal_period();
  double mult2 = spec_.vertical_period();
  double w = ampl1 / 4.0;
  double factor = 1.5 * (w) + 0.9 * (1 - w);

  double ds_dt1 =
      sqrt( pow(A / (2 * sqrt(t)) + ampl1 * mult1 * cos(mult1 * t), 2) +
            pow(A * sqrt(t) + ampl1 * sin(mult1 * t), 2) +
            pow(2 * B * t + ampl2 * mult2 * cos(mult2 * t), 2));
  double dr_dt = - beta * sphere_radius;

  double delta_t1 = factor * sphere_radius / (ds_dt1 - factor * dr_dt / 2);

  double new_t = t + delta_t1;
  double ds_dt2 =
      sqrt( pow(A / (2 * sqrt(new_t)) +
                ampl1 * mult1 * cos(mult1 * new_t), 2) +
            pow(A * sqrt(new_t) + ampl1 * sin(mult1 * new_t), 2) +
            pow(2 * B * new_t + ampl2 * mult2 * cos(mult2 * new_t), 2));

  double ds_dt = (ds_dt1 + ds_dt2) / 2.0;
  double delta_t = factor * sphere_radius / (ds_dt - factor * dr_dt / 2);

  if (t + delta_t < max_t && delta_t > 0.005) {
    add_transform_recursive(
        max_t, A, B, ampl1, ampl2, alpha, beta, t + delta_t,
        it_num + 1, transforms);
  }
}

void TurnIntoTentacleOperation::GetTransforms(
    vector<Eigen::Matrix4d>* transforms) const {
  double A = spec_.a();
  double start_radius = spec_.start_radius();
  double end_radius = spec_.end_radius();
  double height = spec_.height();
  double n_revs = spec_.n_revs();
  double waviness = spec_.waviness();
  double ampl1 = 1 * waviness;
  double ampl2 = 1.5 * waviness;
  double start_t = 0.01; //0.2;

  double max_t = 2 * std::acos(-1) * n_revs + start_t;
  double B = height / pow(max_t, 2);

  // Exponential radius decay
  double beta = std::log(start_radius / end_radius) / (max_t - start_t);
  double alpha = start_radius / exp(- beta * start_t);

  add_transform_recursive(
      max_t, spec_.a(), B, ampl1, ampl2, alpha, beta, start_t, 0, transforms);
}

void TurnIntoTentacleOperation::Modify(Shape* shape) {
  if (spec_.empty_if_flat() && spec_.height() <= 0.0001) {
    shape->Reset(Shape());
    return;
  }
  shape->Mesh().needBoundingBox();
  TriMesh::Point dif = shape->Mesh().bounding_box_max() - shape->Mesh().bounding_box_min();
  double min_dim = std::min(fabs(dif[0]), std::min(fabs(dif[1]), fabs(dif[2])));
  if (min_dim > 0.00001) {
    base_scale_ = 2.0 / min_dim;
  } else {
    base_scale_ = 1.0;
  }

  vector<Eigen::Matrix4d> transforms;
  GetTransforms(&transforms);

  if (transforms.empty()) {
    LOG(ERROR) << "No shapes produced in TENTACLE; doing nothing";
    return;
  }

  vector<Shape*> shapes;
  int skipped = 0;
  for (const auto& trans : transforms) {
    if (skipped >= spec_.skip_first_n()) {
      shapes.push_back(new Shape(shape->Mesh()));
      shapes.back()->AddTransform(trans);
    }
    ++skipped;
  }

  if (shapes.size() == 0) {
    LOG(ERROR) << "No tentacle links generated.";
  } else if (shapes.size() == 1) {
    shape->Reset(*shapes[0]);
  } else if (FLAGS_debug_turn_into_tentacle) {
    for (int i = 1; i < shapes.size(); ++i) {
      shapes[0]->MutableMesh().AddData(shapes[i]->Mesh());
    }
    shape->Reset(*shapes[0]);
  } else {
    carve::csg::CSG csg_util;

    boost::scoped_ptr<carve::poly::Polyhedron> res(
        csg_util.compute(
            &shapes[0]->Polyhedron(), &shapes[1]->Polyhedron(),
            carve::csg::CSG::UNION));
    for (int i = 2; i < shapes.size(); ++i) {
      carve::poly::Polyhedron* tmp_res =
          csg_util.compute(
              res.get(), &shapes[i]->Polyhedron(),
              carve::csg::CSG::UNION);
      res.reset(tmp_res);
    }
    Shape::PolyhedronToMesh(*res, &shape->MutableMesh());
  }
  for (int i = 0; i < shapes.size(); ++i) {
    delete shapes[i];
  }
}

}  // namespace mit_plato
