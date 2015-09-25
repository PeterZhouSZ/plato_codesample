#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS__COMBINE_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS__COMBINE_OPERATIONS_H__

#include "fab/geometry/template/operations.h"
#include "fab/geometry/template/operations.pb.h"

namespace mit_plato {

class UnionOperation : public CombineShapesOperation {
 public:
  UnionOperation() {
    ExposeParams(&spec_);
  }

  UnionOperation(const UnionOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual Shape* Combine(const vector<const Shape*>& shapes);

 private:
  UnionOpSpec spec_;
};

class IntersectOperation : public CombineShapesOperation {
 public:
  IntersectOperation() {
    ExposeParams(&spec_);
  }

  IntersectOperation(const IntersectOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual Shape* Combine(const vector<const Shape*>& shapes);

 private:
  IntersectOpSpec spec_;
};

class SubtractOperation : public CombineShapesOperation {
 public:
  SubtractOperation() {
    ExposeParams(&spec_);
  }

  SubtractOperation(const SubtractOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual Shape* Combine(const vector<const Shape*>& shapes);

 private:
  SubtractOpSpec spec_;
};

class SimpleCombineOperation : public CombineShapesOperation {
 public:
  SimpleCombineOperation() {
    ExposeParams(&spec_);
  }

  SimpleCombineOperation(const SimpleCombineOpSpec& spec) {
    spec_.CopyFrom(spec);
    ExposeParams(&spec_);
  }

  virtual Shape* Combine(const vector<const Shape*>& shapes);

 private:
  SimpleCombineOpSpec spec_;
};

}  // namespace mit_plato

#endif  //_FAB_GEOMETRY_TEMPLATE_OPERATIONS__COMBINE_OPERATIONS_H__
