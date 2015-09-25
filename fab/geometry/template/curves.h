#ifndef _FAB_GEOMETRY_TEMPLATE_CURVES_H__
#define _FAB_GEOMETRY_TEMPLATE_CURVES_H__

#include <set>
#include <string>
#include <vector>
using std::set;
using std::string;
using std::vector;

#include <Eigen/Dense>
using namespace Eigen;

#include <glog/logging.h>

#include "fab/geometry/template/operations.pb.h"


namespace mit_plato {

//!
//! General interface for all 3d parametrized curves.
class ParameterizedCurve {
 public:
  ParameterizedCurve();

  virtual ~ParameterizedCurve() {}

  //!
  //! Copies the curve by writing it to spec and reading it from spec;
  //! caller responsible for memory.
  static ParameterizedCurve* Copy(const ParameterizedCurve& curve);

  //! Creates a curve subclass from spec; see implementation for
  //! supported types.
  // TODO: add registration
  static ParameterizedCurve* FromSpec(const RegisteredCurveSpec& spec);

  //!
  //! Writes the curve to spec. Calls subclass ToSpecInternal and fills in
  //! curve-level information such as transforms.
  bool ToSpec(RegisteredCurveSpec* spec) const;

  //!
  //! Returns true if the curve is closed.
  bool is_closed() const;

  //! Returns position parametrized by t in [0, 1]. If t is
  //! outside of the expected bounds, its value should be capped
  //! to the valid range by the subclasses.
  //! As a rule, planar curves in ZX plane are counterclockwise:
  //!     z
  //!     |
  //! ----|--- x
  //!     |
  //!     |
  Vector3d Position(double t) const;

  //! Returns a sequence of positions representing the whole curve
  //! as a vector; if the curve is closed, last position, identical to,
  //! the first, will be omitted.
  //! TODO: adaptively sample the curve
  virtual bool Discretize(vector<Vector3d>* positions) const;

  //!
  //! Adds transform to all point positions.
  void AddTransform(const Matrix4d& transform);

  //!
  //! Clears all transforms that have not been baked in.
  void ClearTransforms();

  //!
  //! Makes all currently set transforms permanent, virtual because
  //! subclasses may choose to e.g. apply baked in transforms to
  //! control pts or other internal representation directly.
  virtual void BakeInTransforms();

  //! Returns any salient points that e.g. have a discontinuity
  //! and should be sampled when sampling points on the curve
  //! to create geometry.
  virtual void GetSalientPoints(set<double>* pts) const {
    pts->insert(0.0);
    pts->insert(1.0);
  }

 protected:
  //!
  //! Writes the curve to spec, must be implemented by each subclass.
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const = 0;

  //!
  //! Subclasses must implement this method, which returns raw position
  //! without any transforms.
  virtual Vector3d PositionInternal(double t) const = 0;

  // Caps t to 0, 1 range, emitting a warning if the value was invalid
  double CapTimeToValidRange(double t) const;

  static bool DeserializeMatrix(const string& encoded, Matrix4d* mat);
  static bool SerializeMatrix(const Matrix4d& mat, string* encoded);

 private:
  // Temporary transforms that can be cleared
  Matrix4d transform_;

  // Baked-in permanent transforms
  Matrix4d permanent_transform_;
};


// TODO: handle arbitrary planes; currently is just in the xz plane
class ParameterizedCircle : public ParameterizedCurve {
 public:
  ParameterizedCircle(double radius = 1.0);

  double radius() const { return r_; }

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  double r_;
};

// TODO: handle arbitrary planes; currently is just in the xz plane
class ParameterizedOval : public ParameterizedCurve {
 public:
  ParameterizedOval(double radius1 = 1.0, double radius2 = 1.5);

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  double r1_;
  double r2_;
};


class ParameterizedYinYang : public ParameterizedCurve {
 public:
  ParameterizedYinYang();

  virtual void GetSalientPoints(set<double>* pts) const;
  virtual bool Discretize(vector<Vector3d>* positions) const;

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  ParameterizedCircle large_circle_;
  ParameterizedCircle small_circle_;
};


class ParameterizedSineWave : public ParameterizedCurve {
 public:
  ParameterizedSineWave(const CreateSineWaveSpec& spec);

  virtual void GetSalientPoints(set<double>* pts) const;
  virtual bool Discretize(vector<Vector3d>* positions) const;

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  CreateSineWaveSpec spec_;
};


class ParameterizedPolygon : public ParameterizedCurve {
 public:
  ParameterizedPolygon(int nsides = 4, double radius = 1.0)
      : nsides_(nsides), circle_(radius) {}

  virtual void GetSalientPoints(set<double>* pts) const;

  virtual bool Discretize(vector<Vector3d>* positions) const;

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  int nsides_;
  ParameterizedCircle circle_;
};

// Implements cubic hermite interpolation between any number of points
// each with 2 gradients, e.g.:
//
//         pt3
//          *\ 1st gradient
//           :\
//            :
//            :| 2nd gradient
//             * pt2
//            :| 1st gradient
//           :
//          :/ 2nd gradient
//          * pt1
class HermiteCurve : public ParameterizedCurve {
 public:
  HermiteCurve() {}

  HermiteCurve(const CreateHermiteSpec& spec);

  //! Removes last point
  void RemovePoint();

  void AddPoint(const Vector3d& position);

  void AddPoint(const Vector3d& position,
                const Vector3d& gradient,
                const Vector3d& second_gradient);
  void Reset();

  virtual void GetSalientPoints(set<double>* pts) const;

  virtual bool Discretize(vector<Vector3d>* positions) const;

  // Forces the curve to recompute gradients based on the
  // Catmull-Rom spline criterion.
  void force_catmull_rom(const double tension);

  void ResetPointToCatmullRom(const int id,
                              const double tension);

  const vector<Vector3d>& points() const { return points_; }

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const;

 private:
  // [p_i g_i p_{i+1} g_{i+1}]
  MatrixXd ControlsMatrix(int index) const;

  // [t^3 t^2 t 1]
  Vector4d TimeVector(double t) const;

  // Matrix for computing blending functions
  static Matrix4d HermiteMatrix();

  void ResetGradientsToCatmullRom();

  static const Matrix4d hermite_;

  vector<Vector3d> points_;
  vector<vector<Vector3d> > gradients_;
  CreateHermiteSpec spec_;
};

class ImageCurve : public ParameterizedCurve {
 public:
  ImageCurve(const CreateImageCurveSpec& spec);

  virtual void GetSalientPoints(set<double>* pts) const {
    return curve_.GetSalientPoints(pts);
  }

 protected:
  virtual bool ToSpecInternal(RegisteredCurveSpec* spec) const;
  virtual Vector3d PositionInternal(double t) const {
    return curve_.Position(t);
  }

 private:
  ImageCurve() {}

  CreateImageCurveSpec spec_;
  HermiteCurve curve_;
};


}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CURVES_H__
