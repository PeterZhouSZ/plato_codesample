#ifndef _UTIL_STL_UTIL__
#define _UTIL_STL_UTIL__

#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>

namespace mds {

inline std::string JoinStrings(const std::vector<std::string>& strs,
                               const std::string& delim) {
  return boost::algorithm::join(strs, delim);
}

template <class T>
inline std::string ContainerJoin(
    const T& container, const std::string& delim) {
  std::stringstream ss;
  for (const auto& val : container) {
    ss << val << delim;
  }
  return ss.str();
}

template <class T, class V>
inline bool Contains(const T& container, const V& val) {
  return std::find(container.begin(), container.end(), val)
      != container.end();
}

template <class K, class V>
inline V CheckGetValue(const std::map<K, V>& m,
                       const K key) {
  typename std::map<K, V>::const_iterator it = m.find(key);
  CHECK(it != m.end()) << "Key not found.";
  return it->second;
}

template <class K, class V>
inline V& CheckGetValueMut(std::map<K, V>& m,
                           const K key) {
  typename std::map<K, V>::iterator it = m.find(key);
  CHECK(it != m.end()) << "Key not found.";
  return it->second;
}

}  // namespace mds

#endif  // _UTIL_STL_UTIL__
