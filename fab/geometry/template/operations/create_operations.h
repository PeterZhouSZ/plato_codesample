#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS__CREATE_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS__CREATE_OPERATIONS_H__


#include <map>
using std::map;
#include <set>
using std::set;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <Eigen/Dense>

#include "fab/geometry/template/curves.h"
#include "fab/geometry/template/operations.h"
#include "fab/geometry/tri_mesh.h"

class CreateRevolvedCurveTest;

namespace mit_plato {

class TemplateEditor;

class ImportMeshOperation : public CreateShapeOperation {
 public:
  ImportMeshOperation() {}  // TODO: add logic
  explicit ImportMeshOperation(const ImportMeshOpSpec& spec);

 protected:
  virtual Shape* CreateUncached();

 private:
  ImportMeshOpSpec spec_;
};

class BoxFluidSimOperation : public CreateShapeOperation {
 public:
  BoxFluidSimOperation() {}  // TODO: add logic
  explicit BoxFluidSimOperation(const BoxFluidSimOpSpec& spec);

 protected:
  virtual Shape* CreateUncached();

 private:
  string FormatFilename() const;
  void MockComputationTime() const;
  void ReadTimings();

  BoxFluidSimOpSpec spec_;
  map<int32, double> timings_;
};

class CreateOffsetCurveOperation : public CreateShapeOperation {
 public:
  CreateOffsetCurveOperation() {}  // TODO: add logic
  explicit CreateOffsetCurveOperation(const CreateOffsetCurveOpSpec& spec);

 protected:
  virtual Shape* CreateUncached();

 private:
  CreateOffsetCurveOpSpec spec_;
  boost::scoped_ptr<ParameterizedCurve> curve_;
};


class CreateRevolvedCurveOperation : public CreateShapeOperation {
 public:
  friend class CreateRevolvedCurveTest;

  CreateRevolvedCurveOperation() {}  // TODO: add logic
  explicit CreateRevolvedCurveOperation(const CreateRevolvedCurveOpSpec& spec);
  virtual void Reset();

 protected:
  virtual Shape* CreateUncached();

  void SetCurvesForTesting(ParameterizedCurve* base,
                           ParameterizedCurve* curve);
 private:
  static void AlignCurveToBase(const Eigen::Vector3d& base_start,
                               ParameterizedCurve* curve);

  static void GetBaseCurveRotations(const vector<Eigen::Vector3d>& base_pts,
                                    vector<Eigen::Matrix4d>& rotations,
                                    bool &is_clockwise);

  bool CreateVerticalSurface(
      const vector<Eigen::Vector3d>& base_pts,
      int offset,
      const ParameterizedCurve& vertical_curve,
      bool reverse_triangles,
      TriMesh* mesh,
      vector<TriMesh::VertexHandle>& first_verts,
      vector<TriMesh::VertexHandle>& prev_verts);

  CreateRevolvedCurveOpSpec spec_;
  // Vertical curve
  boost::scoped_ptr<ParameterizedCurve> curve_;
  // Base curve around which the vertical curve is "revolved"
  boost::scoped_ptr<ParameterizedCurve> base_;
};

}  // namespace mit_plato

#endif  //_FAB_GEOMETRY_TEMPLATE_OPERATIONS__CREATE_OPERATIONS_H__
