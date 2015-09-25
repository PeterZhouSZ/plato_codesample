#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS__OPENSCAD_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS__OPENSCAD_OPERATIONS_H__

#include <string>
using std:: string;

#include <boost/thread/mutex.hpp>

#include "fab/geometry/template/operations/create_operations.h"
#include "fab/geometry/template/operations.pb.h"

namespace mit_plato {

class Shape;

// Singleton class to enable Openscad support
class OpenscadSupport {
 public:
  static void Enable();

 private:
  OpenscadSupport() : enabled_(false) {}
  static OpenscadSupport& Singleton();

  bool enabled_;
  boost::mutex lock_;
};

//!
//! Creates geometry from an openscad script with special
//! $1, $2, etc. variables.
class OpenscadCreateOperation : public CreateShapeOperation {
 public:
  OpenscadCreateOperation() {}  // TODO: add logic
  virtual ~OpenscadCreateOperation() {}
  explicit OpenscadCreateOperation(const OpenscadCreateOpSpec& spec);

  virtual const NamedVarContainer& params() const { return program_params_; }
  virtual NamedVarContainer& params() { return program_params_; }

 protected:
  virtual Shape* CreateUncached();

  void Reset();

 private:
  string program_;

  OpenscadCreateOpSpec spec_;
  SimpleContainer program_params_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_OPERATIONS__OPENSCAD_OPERATIONS_H__
