#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_BOUNDS_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_BOUNDS_H__

#include <vector>
#include "fab/geometry/template/template.pb.h"

namespace mit_plato {

template <class T>
class BoundInterval {
 public:
  BoundInterval(const T& mi, const T& ma) : min_(mi), max_(ma) {
    CHECK_LE(mi, ma);
  }

  bool Contains(const T& val) const {
    return val >= min_ && val <= max_;
  }

  bool Overlaps(const BoundInterval<T>& other) const {
    return Contains(other.min_) || Contains(other.max_);
  }

  const T& min_val() const { return min_; }

  const T& max_val() const { return max_; }

 private:
  T min_;
  T max_;
  BoundInterval() {}  // disallow
};

class Bounds {
 public:
  virtual ~Bounds() {}

  Bounds* Copy() const {
    BoundsSpec s;
    ToSpec(&s);
    return FromSpec(s);
  }

  virtual bool ToSpec(BoundsSpec* spec) const = 0;

  static Bounds* FromSpec(const BoundsSpec& spec);
};

std::ostream& operator<<(std::ostream& os,
                         const Bounds& bounds);

class TrivialBounds : public Bounds {
 public:
  virtual bool ToSpec(BoundsSpec* spec) const { return true; }
};

class BoolBounds : public Bounds {
 public:
  BoolBounds(bool allowed_value);
  BoolBounds(const BoundsSpec::BoolSpec& spec)
      : allowed_val(spec.allowed_value()) {}

  virtual bool ToSpec(BoundsSpec* spec) const;

  bool IsValid(bool val) const { return val == allowed_val; }

  bool allowed_val;

 private:
  BoolBounds() {}
};


class IntBounds : public Bounds {
 public:
  IntBounds(const int mi = 0,
            const int ma = 100);

  IntBounds(const BoundsSpec::IntSpec& spec);

  virtual bool ToSpec(BoundsSpec* spec) const;

  bool IsValid(int val) const;

  const std::vector<BoundInterval<int> >& intervals() const {
    return intervals_;
  }

 private:
  std::vector<BoundInterval<int> > intervals_;

  IntBounds() {}
};


class DoubleBounds : public Bounds {
 public:
  DoubleBounds(const double mi,
               const double ma);

  DoubleBounds(const BoundsSpec::DoubleSpec& spec);

  virtual bool ToSpec(BoundsSpec* spec) const;

  bool IsValid(double val) const;

  const std::vector<BoundInterval<double> >& intervals() const {
    return intervals_;
  }

 private:
  std::vector<BoundInterval<double> > intervals_;

  DoubleBounds() {}
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_BOUNDS_H__
