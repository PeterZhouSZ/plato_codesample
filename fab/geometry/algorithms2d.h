#ifndef _FAB_GEOMETRY_ALGORITHMS_2D_H__
#define _FAB_GEOMETRY_ALGORITHMS_2D_H__

#include <vector>
#include <Eigen/Dense>

namespace mds {
namespace algo_2d {

//! Used for indexing into vectors of various representations;
//! allows converting from 3d vectors in the plane to 2d vectors
//! without explicitly copying out 2d data.
//!
//! E.g. if want to operate on polygons in the ZX plane in the
//! right-handed cartesian system, use Indexer(2, 0).
struct Indexer {
 public:
  Indexer(int x_idx = 0, int y_idx = 1) : x_(x_idx), y_(y_idx) {}

  Indexer(const Indexer& other) : x_(other.x_), y_(other.y_) {}

  template <class V>
  double x(const V& vec) const { return vec[x_]; }

  template <class V>
  double y(const V& vec) const { return vec[y_]; }

  template <class V>
  void set_x(V* vec, const double x) const { (*vec)[x_] = x; }

  template <class V>
  void set_y(V* vec, const double y) const { (*vec)[y_] = y; }

 private:
  int x_;
  int y_;
};

//!
//! Line equation of the form Ax + By = C
struct ABCLineEq {
  ABCLineEq() : A(0), B(0), C(0) {}

  // Returns true if all coefficients are zero
  bool IsNull() const;

  double A;
  double B;
  double C;
};


//! Aligns two arrays of points representing polygons so that
//! the sum of pairwise squared distances between points is minimized.
//! (we need this mostly to align polygon offset with the polygon).
//! Uses a stupid n^2 algorithm.
class PolygonPtAligner {
 public:
  //! Keeps a ptr to to_align
  PolygonPtAligner(const std::vector<Eigen::Vector3d>* to_align,
                   const Indexer& ider);

  //! Runs actual alignment with reference; reference must have
  //! the same size as to_align, otherwise returns false.
  bool AlignTo(const std::vector<Eigen::Vector3d>& reference);

  //! Returns the to_align element that's been aligned with the
  //! reference, or possibly an interpolated version.
  const Eigen::Vector3d& aligned(int idx) const;

 private:
  PolygonPtAligner() {}

  // Resets past alignment info
  void Reset();

  // Produces optimal alignment; works only when aligning equal # pts
  bool ExactAlign(const std::vector<Eigen::Vector3d>& reference);

  // Greedy alignment algorithm; works even when reference has more points
  bool GreedyInterpolatingAlign(const std::vector<Eigen::Vector3d>& reference);

  bool GreedyInterpolatingAlign(const std::vector<Eigen::Vector3d>& reference,
                                const std::vector<Eigen::Vector3d>& to_align,
                                std::vector<Eigen::Vector3d>& aligned,
                                double* cost) const;

  const Eigen::Vector3d& offset_data(int idx, int offset, bool mirror) const;

  bool update_min_dist(const std::vector<Eigen::Vector3d>& reference,
                       int offset, bool mirror,
                       double* current_min) const;

  int closest_pt_idx(const std::vector<Eigen::Vector3d>& reference,
                     const Eigen::Vector3d& pt,
                     int& search_start,
                     int& search_end) const;

  double planar_dist_sq(const Eigen::Vector3d& v1,
                        const Eigen::Vector3d& v2) const;

  // DATA ---------------------
  const std::vector<Eigen::Vector3d>* to_align_;
  const Indexer ider_;

  // Filled in if using exact alignment
  int best_offset_;
  bool mirror_;

  // Filled in if using greedy align strategy
  std::vector<Eigen::Vector3d> aligned_;
};

//!
//! Gets line equation for line passing through 2 points.
ABCLineEq GetLineEquation(const Eigen::Vector3d& v1,
                          const Eigen::Vector3d& v2,
                          const Indexer& ider);

//!
//! Gets intersection of two lines, returns false if lines are
//! parallel or singular.
bool IntersectLines(const ABCLineEq& line1,
                    const ABCLineEq& line2,
                    const Indexer& ider,
                    Eigen::Vector3d* intercept);

//!
//! Computes inner offset of the polygon by a given thickness,
//! returns 3d points with the 3rd coordinate value (i.e. one
//! not mapped to XY by indexer) set to its value in the first
//! point.
bool InnerOffsetPolygon(const std::vector<Eigen::Vector3d>& pts,
                        const Indexer& ider,
                        double thickness,
                        std::vector<Eigen::Vector3d>* out_pts);

//!
//! Splits segments until no segment is more than twice the length
//! of another; also ensures that this results in at least min
//! total number of segments.
void EqualizeSegmentLengths(const std::vector<Eigen::Vector3d>& points,
                            const int min_segment_num,
                            std::vector<Eigen::Vector3d>* out_pts);

//!
//! Returns true if the point is inside the triangle.
template <class V, class P>
bool IsPointInsideTriangle(const V& v1,
                           const V& v2,
                           const V& v3,
                           const P& pt,
                           const Indexer& ider) {
  Eigen::Matrix2d A;
  A << ider.x(v3) - ider.x(v1), ider.x(v2) - ider.x(v1),
      ider.y(v3) - ider.y(v1), ider.y(v2) - ider.y(v1);
  Eigen::Vector2d b;
  b << ider.x(pt) - ider.x(v1),
      ider.y(pt) - ider.y(v1);
  Eigen::Vector2d x = A.inverse() * b;  // assume invertible
  return x[0] >= 0 && x[0] <= 1.0 &&
      x[1] >= 0 && x[1] <= 1.0;
}

}  // namespace algo_2d
}  // namespace mds

#endif  // _FAB_GEOMETRY_ALGORITHMS_2D_H__
