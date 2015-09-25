#ifndef _FAB_GEOMETRY_TEMPLATE_CHECKS_H__
#define _FAB_GEOMETRY_TEMPLATE_CHECKS_H__

#include <stdexcept>
#include <string>
#include <vector>
using std::vector;
using std::string;
using std::runtime_error;

#include <Eigen/Dense>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/variate_generator.hpp>
#include <google/protobuf/stubs/common.h>
using google::protobuf::int32;

#include "fab/geometry/template/bounds.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/template/template.pb.h"

namespace mit_plato {

class Shape;

// Wrapper for tests of design validity
class ShapeChecker {
 public:
  class BoundingBox {
   public:
    BoundingBox(const Eigen::Vector3d& min = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& max = Eigen::Vector3d::Zero())
        : min_(min),
          max_(max) {}

    void Reset(const Eigen::Vector3d& min,
               const Eigen::Vector3d& max) {
      min_ = min;
      max_ = max;
    }

    Eigen::Vector3d UniformSample(
        boost::variate_generator <boost::mt19937,
        boost::uniform_01<> >& generator) const;

    const Eigen::Vector3d& min() const { return min_;}
    const Eigen::Vector3d& max() const { return max_;}

   private:
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
  };

  class NeighborsCube {
   public:
    NeighborsCube(Shape::GridType::ConstPtr volume,
                  openvdb::Coord index);

    Eigen::Vector3d ComputeGradient() const;

   private:
    double vals_[3][3][3];
  };

  static bool IsWatertight(const Shape& shape);


  static double MaterialVolume(const Shape& shape);

  static double ShapeDiameter(const Shape& shape);

  static bool IsStable(
      const Shape& shape,
      const Eigen::Vector3d& orientation = Eigen::Vector3d(0, 1, 0));

  static Eigen::Vector3d CenterOfMass(const Shape& shape);

  static bool IsInside(const Shape& shape,
                       const Eigen::Vector3d& v,
                       double min_dist_from_shell = 0);

  static Eigen::Vector3d GetDepthGradient(const Shape& shape,
                                          const Eigen::Vector3d& pos);

  static double MaxShellThickness(const Shape& shape);

  static void GetBoundingBox(const Shape& shape, BoundingBox* bbox);

  // Returns XOR of the two shapes normalized by the union
  static double GetShapeDeltaNorm(const Shape& s1, const Shape& s2);

 private:
  // For now above operations are only implemented for default orientation
  // of (0, 1, 0).
  static bool IsDefaultOrientation(const Eigen::Vector3d& orientation);
};


class OperationNode;

// Base class for all property evaluators
class PropEvaluator {
 public:
  virtual ~PropEvaluator() {}

  // Creates a specialized PropEvaluator subclass from spec
  static PropEvaluator* FromSpec(const PropEvaluatorSpec& spec);

  virtual bool ToSpec(PropEvaluatorSpec* spec) const = 0;

  virtual VarInfoSpec::VarType type() const = 0;

  virtual const string& id() const = 0;

  // Note: may have to change the signature of this to allow
  // lazy re-evaluation of properties at the node level;
  // for now we don't worry about that
  virtual bool EvaluateBool(OperationNode* node, bool* is_valid) const {
    vector<OperationNode*> nodes;
    nodes.push_back(node);
    return EvaluateBool(nodes, is_valid);
  }

  virtual double EvaluateDouble(OperationNode* node, bool* is_valid) const {
    vector<OperationNode*> nodes;
    nodes.push_back(node);
    return EvaluateDouble(nodes, is_valid);
  }

  virtual int32 EvaluateInt(OperationNode* node, bool* is_valid) const {
    vector<OperationNode*> nodes;
    nodes.push_back(node);
    return EvaluateInt(nodes, is_valid);
  }

  virtual bool EvaluateBool(const vector<OperationNode*>& nodes,
                            bool* is_valid) const {
    throw runtime_error("Not implemented");
  }

  virtual double EvaluateDouble(const vector<OperationNode*>& nodes,
                                bool* is_valid) const {
    throw runtime_error("Not implemented");
  }

  virtual int32 EvaluateInt(const vector<OperationNode*>& nodes,
                            bool* is_valid) const {
    throw runtime_error("Not implemented");
  }
};


class WatertightPropEvaluator : public PropEvaluator {
 public:
  static const string registered_name;

  WatertightPropEvaluator(const PropEvaluatorSpec& spec);
  virtual bool ToSpec(PropEvaluatorSpec* spec) const;

  virtual VarInfoSpec::VarType type() const { return VarInfoSpec::TYPE_BOOL; }

  virtual const string& id() const { return registered_name; }

  virtual bool EvaluateBool(const vector<OperationNode*>& nodes,
                            bool* is_valid) const;

 private:
  boost::scoped_ptr<BoolBounds> bounds_;
};


class IsStablePropEvaluator : public PropEvaluator {
 public:
  static const string registered_name;

  IsStablePropEvaluator(const PropEvaluatorSpec& spec);
  virtual bool ToSpec(PropEvaluatorSpec* spec) const;

  virtual VarInfoSpec::VarType type() const { return VarInfoSpec::TYPE_BOOL; }

  virtual const string& id() const { return registered_name; }

  virtual bool EvaluateBool(const vector<OperationNode*>& nodes,
                            bool* is_valid) const;

 private:
  boost::scoped_ptr<BoolBounds> bounds_;
};


class MaterialVolumePropEvaluator : public PropEvaluator {
 public:
  static const string registered_name;

  MaterialVolumePropEvaluator(const PropEvaluatorSpec& spec);
  virtual bool ToSpec(PropEvaluatorSpec* spec) const;

  virtual VarInfoSpec::VarType type() const { return VarInfoSpec::TYPE_DOUBLE; }

  virtual const string& id() const { return registered_name; }

  virtual double EvaluateDouble(const vector<OperationNode*>& nodes,
                                bool* is_valid) const;

 private:
  boost::scoped_ptr<DoubleBounds> bounds_;
};

class ShapeDiameterPropEvaluator : public PropEvaluator {
 public:
  static const string registered_name;

  ShapeDiameterPropEvaluator(const PropEvaluatorSpec& spec);
  virtual bool ToSpec(PropEvaluatorSpec* spec) const;

  virtual VarInfoSpec::VarType type() const { return VarInfoSpec::TYPE_DOUBLE; }

  virtual const string& id() const { return registered_name; }

  virtual double EvaluateDouble(const vector<OperationNode*>& nodes,
                                bool* is_valid) const;

 private:
  boost::scoped_ptr<DoubleBounds> bounds_;
};

class ShoeSimulationPropEvaluator : public PropEvaluator {
 public:
  static const string registered_name;

  ShoeSimulationPropEvaluator(const PropEvaluatorSpec& spec);
  virtual bool ToSpec(PropEvaluatorSpec* spec) const;

  virtual VarInfoSpec::VarType type() const { return VarInfoSpec::TYPE_DOUBLE; }

  virtual const string& id() const { return registered_name; }

  virtual double EvaluateDouble(const vector<OperationNode*>& nodes,
                                bool* is_valid) const;

 private:
  boost::scoped_ptr<DoubleBounds> bounds_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CHECKS_H__
