#include <stdexcept>
using std::runtime_error;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <openvdb/tools/Composite.h>
#include <carve/polyhedron_decl.hpp>
#include <carve/csg.hpp>

#include "fab/geometry/template/operations/combine_operations.h"
#include "fab/geometry/template/registration.h"

DEFINE_bool(use_volumetric_csg, false,
            "If set uses OpenVDB for CSG operations; "
            "else uses Carve library.");
DEFINE_bool(volumetric_csg_fallback, false,
            "If true, allows falling back to volumetric CSG if "
             "mesh CSG fails.");

namespace mit_plato {

Shape* UnionOperation::Combine(const vector<const Shape*>& shapes) {
  if (shapes.empty()) return NULL;
  if (shapes.size() == 1) {
    LOG(WARNING) << "Combining only one shape; makes no sense.";
    return new Shape(*shapes[0]);
  }

  Shape* combined = NULL;
  try {
    if (!FLAGS_use_volumetric_csg) {
      try {
        combined = new Shape();
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
        Shape::PolyhedronToMesh(*res, &combined->MutableMesh());
      } catch (carve::exception& e) {
        delete combined;

        LOG(ERROR) << "Caught exception while performing Carve CSG: " << e.str();
        if (!FLAGS_volumetric_csg_fallback) {
          throw e;  // not caught
        } else {
          throw runtime_error(
              "Caught exception while performing Carve CSG: " + e.str());
        }
      } catch (...) {
        delete combined;
        LOG(ERROR) << "Caught Unknown exception while performing Carve CSG";
        throw;
      }
    } else {
      throw runtime_error("Use volumetric CSG");
    }
  } catch (runtime_error& e) {
    if (!FLAGS_volumetric_csg_fallback && !FLAGS_use_volumetric_csg) {
      throw e;
    }
    VLOG(1) << "Using volumetric CSG";
    combined = new Shape();
    for (int i = 0; i < shapes.size(); ++i) {
      VLOG(3) << "Union, adding shape with: ";
      shapes[i]->log_info();
      openvdb::tools::csgUnion(
          *combined->MutableVolume(),
          *shapes[i]->VolumeCopy());
      VLOG(3) << "Union, addED shape with: ";
      shapes[i]->log_info();
      VLOG(3) << "Union, accumulated shape with: ";
      combined->log_info();
    }
  }

  return combined;
}
REGISTER_OPERATION_WITH_DEFAULT("Union shapes", UnionOperation, UnionOpSpec);


Shape* IntersectOperation::Combine(const vector<const Shape*>& shapes) {
  if (shapes.empty()) return NULL;
  if (shapes.size() != 2) {
    LOG(WARNING) << "Intersect ill-defined for " << shapes.size()
                 << " number of shapes.";
    return NULL;
  }

  Shape* combined = NULL;
  try {
    if (!FLAGS_use_volumetric_csg) {
      try {
        combined = new Shape();
        carve::csg::CSG csg_util;

        boost::scoped_ptr<carve::poly::Polyhedron> res(
            csg_util.compute(
                &shapes[0]->Polyhedron(), &shapes[1]->Polyhedron(),
                carve::csg::CSG::INTERSECTION));
        Shape::PolyhedronToMesh(*res, &combined->MutableMesh());
      } catch (carve::exception& e) {
        delete combined;

        LOG(ERROR) << "Caught exception while performing Carve CSG: " << e.str();
        if (!FLAGS_volumetric_csg_fallback) {
          throw e;  // not caught
        } else {
          throw runtime_error(
              "Caught exception while performing Carve CSG: " + e.str());
        }
      } catch (...) {
        LOG(ERROR) << "Caught Unknown exception while performing Carve CSG";
        delete combined;
        throw;
      }
    } else {
      throw runtime_error("Use volumetric CSG");
    }
  } catch (runtime_error& e) {
    if (!FLAGS_volumetric_csg_fallback && !FLAGS_use_volumetric_csg) {
      throw e;
    }
    VLOG(1) << "Using volumetric CSG";
    combined = new Shape(*shapes[0]);
    openvdb::tools::csgIntersection(
        *combined->MutableVolume(),
        *shapes[1]->VolumeCopy());
  }

  return combined;
}
REGISTER_OPERATION_WITH_DEFAULT("Intersect shapes", IntersectOperation, IntersectOpSpec);


Shape* SubtractOperation::Combine(const vector<const Shape*>& shapes) {
  if (shapes.empty()) return NULL;
  if (shapes.size() == 1) {
    LOG(WARNING) << "Combining only one shape; makes no sense.";
    return new Shape(*shapes[0]);
  }

  Shape* combined = NULL;
  try {
    if (!FLAGS_use_volumetric_csg) {
      try {
        combined = new Shape();
        carve::csg::CSG csg_util;

        boost::scoped_ptr<carve::poly::Polyhedron> res(
            csg_util.compute(
                &shapes[0]->Polyhedron(), &shapes[1]->Polyhedron(),
                carve::csg::CSG::A_MINUS_B));
        for (int i = 2; i < shapes.size(); ++i) {
          carve::poly::Polyhedron* tmp_res =
              csg_util.compute(
                  res.get(), &shapes[i]->Polyhedron(),
                  carve::csg::CSG::A_MINUS_B);
          res.reset(tmp_res);
        }
        Shape::PolyhedronToMesh(*res, &combined->MutableMesh());
      } catch (carve::exception& e) {
        delete combined;

        LOG(ERROR) << "Caught exception while performing Carve CSG: " << e.str();
        if (!FLAGS_volumetric_csg_fallback) {
          throw e;  // not caught
        } else {
          throw runtime_error(
              "Caught exception while performing Carve CSG: " + e.str());
        }
      } catch(...) {
        delete combined;
        LOG(ERROR) << "Caught Unknown exception while performing Carve CSG";
        throw;
      }
    } else {
      throw runtime_error("Use volumetric CSG");
    }
  } catch (runtime_error& e) {
    if (!FLAGS_volumetric_csg_fallback && !FLAGS_use_volumetric_csg) {
      throw e;
    }
    VLOG(1) << "Using volumetric CSG";
    combined = new Shape(*shapes[0]);
    VLOG(1) << "Subtract, subtracting from: ";
    shapes[0]->log_info();
    for (int i = 1; i < shapes.size(); ++i) {
      VLOG(1) << "Subtracting shape with: ";
      shapes[i]->log_info();
      openvdb::tools::csgDifference(*combined->MutableVolume(),
                                    *shapes[i]->VolumeCopy());
      VLOG(1) << "Union, accumulated shape with: ";
      combined->log_info();
    }
  }

  return combined;
}
REGISTER_OPERATION_WITH_DEFAULT("Subtract shapes", SubtractOperation, SubtractOpSpec);


Shape* SimpleCombineOperation::Combine(const vector<const Shape*>& shapes) {
  if (shapes.empty()) return NULL;

  Shape* combined = new Shape();
  for (int i = 0; i < shapes.size(); ++i) {
    combined->MutableMesh().AddData(shapes[i]->Mesh());  // TODO: optimize!!
  }
  return combined;
}
REGISTER_OPERATION(SimpleCombineOperation, SimpleCombineOpSpec);

}  // namespace mit_plato
