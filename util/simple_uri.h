#ifndef _UTIL_COLOR_SIMPLE_URI_H__
#define _UTIL_COLOR_SIMPLE_URI_H__

#include <iostream>
#include <map>
#include <string>

namespace mds_util {

using std::string;
using std::map;

struct SimpleUri {
  SimpleUri(const string& raw) {
    parse(raw);
  }

  void set_path(const string& path) { path_ = path; }
  const string& path() const { return path_; }
  const map<string, string>& query_items() const { return query_items_; }

 private:
  void parse(const string& url_s);

  string path_;
  map<string, string> query_items_;
};

inline std::ostream& operator<<(std::ostream& os,
                                const SimpleUri& uri) {
  os << "path [" << uri.path() << "] query [ ";
  for (const auto& pr : uri.query_items()) {
    os << "{ [" << pr.first << "] = [" << pr.second << "] } ";
  }
  return os;
}


}  // namespace mds_util

#endif  // _UTIL_COLOR_SIMPLE_URI_H__
