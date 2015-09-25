#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS__MODIFY_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS__MODIFY_OPERATIONS_H__

#include <Eigen/Dense>

#include "fab/geometry/template/operations.h"
#include "fab/geometry/template/operations.pb.h"

namespace mit_plato {

class TurnIntoTentacleOperation : public ModifyShapeOperation {
 public:
  TurnIntoTentacleOperation() : base_scale_(1.0) {
    ExposeParams(&spec_);
  }

  TurnIntoTentacleOperation(const TurnIntoTentacleOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  void GetTransforms(vector<Eigen::Matrix4d>* transforms) const;

  void add_transform_recursive(
    double max_t,
    double A,
    double B,
    double ampl1,
    double ampl2,
    double alpha,
    double beta,
    double t,
    double it_num,
    vector<Eigen::Matrix4d>* transforms) const;

  Eigen::Matrix4d one_link(double A,
                           double B,
                           double ampl1,
                           double ampl2,
                           double sphere_radius,
                           double t) const;

  TurnIntoTentacleOpSpec spec_;
  double base_scale_;
};

class SmoothOperation : public ModifyShapeOperation {
 public:
  SmoothOperation() {
    ExposeParams(&spec_);
  }

  // TODO: get rid of code duplication; easy to get wrong
  SmoothOperation(const SmoothOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  SmoothOpSpec spec_;
};


class SubdivideOperation : public ModifyShapeOperation {
 public:
  SubdivideOperation() {
    ExposeParams(&spec_);
  }

  // TODO: get rid of code duplication; easy to get wrong
  SubdivideOperation(const SubdivideOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  SubdivideOpSpec spec_;
};


class RandomDisplacementOperation : public ModifyShapeOperation {
 public:
  RandomDisplacementOperation() {
    ExposeParams(&spec_);
  }

  // TODO: get rid of code duplication; easy to get wrong
  RandomDisplacementOperation(const RandomDisplacementOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  RandomDisplacementOpSpec spec_;
};

class TransformOperation : public ModifyShapeOperation {
 public:
  TransformOperation() {
    ExposeParams(&spec_);
  }

  TransformOperation(const TransformOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  TransformOperation(const Eigen::Matrix4d& mat) {
    ToSpec(mat, &spec_);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  void ToSpec(const Eigen::Matrix4d& mat, TransformOpSpec* spec) const;

  TransformOpSpec spec_;
};

class ScaleOperation : public ModifyShapeOperation {
 public:
  ScaleOperation() {
    ExposeParams(&spec_);
  }

  ScaleOperation(const ScaleOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual void Modify(Shape* shape);

 private:
  ScaleOpSpec spec_;
};

class TranslateOperation : public ModifyShapeOperation {
 public:
  TranslateOperation() {
    ExposeParams(&spec_);
    Reset();
  }

  TranslateOperation(const TranslateOpSpec& spec);

  virtual void Modify(Shape* shape);

  virtual void Reset();

 private:
  TranslateOpSpec spec_;
  Eigen::Matrix4d mat_;
};

class RotateOperation : public ModifyShapeOperation {
 public:
  RotateOperation() {
    ExposeParams(&spec_);
    Reset();
  }

  RotateOperation(const RotateOpSpec& spec);

  virtual void Modify(Shape* shape);

  virtual void Reset();

 private:
  RotateOpSpec spec_;
  Eigen::Matrix4d mat_;
};

}  // namespace mit_plato

#endif  //_FAB_GEOMETRY_TEMPLATE_OPERATIONS__MODIFY_OPERATIONS_H__
