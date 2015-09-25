#ifndef _FAB_GEOMETRY_TEMPLATE_SEARCH_PATHS_H__
#define _FAB_GEOMETRY_TEMPLATE_SEARCH_PATHS_H__

#include <string>
#include <set>
#include <boost/thread/mutex.hpp>

namespace mit_plato {

using std::string;

//! Class responsible for resolving relative paths
class SearchPaths {
 public:
  //!
  //! Adds a search path for use in FindFile.
  static void AddSearchPath(const string& absolute_path);

  //!
  //! If the path is absolute, returns true if the path exists
  //! and sets absolute_path to path.
  //! If the path is relative, searches for an existing path
  //! in all the mesh search paths and returns the first matching
  //! file.
  static bool FindFile(const string& path,
                       string* absolute_path);

  //!
  //! If path is non-empty and non-absolute and --precompute_dir is set,
  //! absolutizes the input path with that directory.
  static void AbsolutizePrecompPath(string* path);


  //!
  //! Same as the other AbsolutizePrecompPath.
  static string AbsolutizePrecompPath(const string& spath);

 private:
  static SearchPaths& Singleton();
  SearchPaths() {}  // disallow

  std::set<string> search_paths_;
  boost::mutex lock_;
};

}  // namespace mit_plato


#endif  // _FAB_GEOMETRY_TEMPLATE_SEARCH_PATHS_H__
