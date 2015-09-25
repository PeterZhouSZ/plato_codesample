#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_AWS_UTIL_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_AWS_UTIL_H__

#include <string>

namespace mit_plato {

using std::string;

// Warning: hangs forever if not authenticated correctly.
// TODO: fix this
class Aws {
 public:
  static bool Cp(const string& file1,
                 const string& file2);

  static bool Mv(const string& file1,
                 const string& file2);

 private:
  static bool BinaryCommand(const string& cmd,
                            const string& file1,
                            const string& file2);

  static bool BinaryCommandWithRetries(const string& cmd,
                                       const string& file1,
                                       const string& file2);
};

}  // namespace mit_plato

#endif
