#ifndef _FAB_GEOMETRY_INTERSECT_H__
#define _FAB_GEOMETRY_INTERSECT_H__

#include <set>
#include <vector>
#include <utility>
#include <Eigen/Dense>

class TriMesh;

namespace mds {

using std::set;
using std::vector;

namespace geo {
struct Ray {
  Ray(const Eigen::Vector3d& origin_in,
      const Eigen::Vector3d& direction_in);

  Eigen::Vector3d origin;
  Eigen::Vector3d direction;
};

inline std::ostream& operator<<(std::ostream& os,
                                const Ray& r) {
  os << "origin: (" << r.origin[0] << ", " << r.origin[1] << ", " << r.origin[2]
     << "), vector: (" << r.direction[0] << ", " << r.direction[1] << ", "
     << r.direction[2] << ")";
  return os;
}

}  // namespace geo


class RayIntersectAccelerator {
 public:
  RayIntersectAccelerator(const TriMesh& mesh,
                          int n_subdiv);

  void IntersectedCells(const geo::Ray& r,
                        set<int>* cells) const;

  void CandidateFaces(const geo::Ray& r,
                      set<unsigned int>* fids) const;

  const vector<unsigned int>& cell_faces(const int cell_id) const {
    return cells_[cell_id];
  }

  int GetSingleIndex(const Eigen::Vector3d& coord) const;


  void LogCellInfo() const;

 private:
  int GetSingleIndexInt(const vector<int>& indices) const;
  bool GetCellIndices(const Eigen::Vector3d& coord,
                      vector<int>* indices) const;

  void AddNeighborhood(const vector<int>& cell,
                       set<int>* cells) const;
  void AddNeighborhood(const vector<int>& cell1,
                       const vector<int>& cell2,
                       set<int>* cells) const;
  bool FindIntersectedCell(const geo::Ray& ray,
                           int coord_index, int coord_value,
                           vector<int>* intersected_cell) const;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  Eigen::Vector3d widths_;

  int n_subdiv_;
  vector<double> cell_width_;

  vector<vector<unsigned int> > cells_;
};

bool RayTriangleIntersect(
    const geo::Ray& ray,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    const Eigen::Vector3d& v3,
    double* distance = NULL);

bool RayMeshIntersect(
    const geo::Ray& ray,
    const TriMesh& mesh,
    std::vector<std::pair<double, unsigned int> >* faces = NULL);

// Accelerated version
bool RayMeshIntersect(
    const geo::Ray& ray,
    const TriMesh& mesh,
    const RayIntersectAccelerator& accel,
    std::vector<std::pair<double, unsigned int> >* faces = NULL);

bool RayPlaneIntersect(const geo::Ray& ray,
                       const geo::Ray& plane,
                       Eigen::Vector3d* int_pt = NULL);


} // namespace mds

#endif  // _FAB_GEOMETRY_INTERSECT_H__
