#include "fab/geometry/template/curves.h"

#include <QColor>
#include <QImage>  // TODO: remove dependency
#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>
#include <boost/math/constants/constants.hpp>
#include <glog/logging.h>

#include "fab/geometry/convert.h"
#include "fab/geometry/template/proto_convert.h"

namespace mit_plato {

using namespace Eigen;

ParameterizedCurve::ParameterizedCurve()
    : transform_(Matrix4d::Identity()),
      permanent_transform_(Matrix4d::Identity()) {}

ParameterizedCurve* ParameterizedCurve::Copy(const ParameterizedCurve& curve) {
  RegisteredCurveSpec spec;
  if (!curve.ToSpec(&spec)) {
    LOG(ERROR) << "Failed to write curve to spec.";
    return NULL;
  }
  return ParameterizedCurve::FromSpec(spec);
}

ParameterizedCurve* ParameterizedCurve::FromSpec(
    const RegisteredCurveSpec& spec) {
  ParameterizedCurve* result = NULL;

  if (spec.HasExtension(CreateCircleSpec::spec)) {
    result = new ParameterizedCircle(
        spec.GetExtension(CreateCircleSpec::spec).radius());
  } else if (spec.HasExtension(CreateOvalSpec::spec)) {
    const CreateOvalSpec& oval_spec =
        spec.GetExtension(CreateOvalSpec::spec);
    result = new ParameterizedOval(oval_spec.radius1(), oval_spec.radius2());
  } else if (spec.HasExtension(CreatePolygonSpec::spec)) {
    const CreatePolygonSpec& curve_spec =
        spec.GetExtension(CreatePolygonSpec::spec);
    result = new ParameterizedPolygon(curve_spec.n_sides(), curve_spec.radius());
  } else if (spec.HasExtension(CreateHermiteSpec::spec)) {
    const CreateHermiteSpec& curve_spec =
        spec.GetExtension(CreateHermiteSpec::spec);
    result = new HermiteCurve(curve_spec);
  } else if (spec.HasExtension(CreateImageCurveSpec::spec)) {
    const CreateImageCurveSpec& curve_spec =
        spec.GetExtension(CreateImageCurveSpec::spec);
    result = new ImageCurve(curve_spec);
  } else if (spec.HasExtension(CreateYinYangCurveSpec::spec)) {
    result = new ParameterizedYinYang();
  } else if (spec.HasExtension(CreateSineWaveSpec::spec)) {
    result = new ParameterizedSineWave(
        spec.GetExtension(CreateSineWaveSpec::spec));
  } else {
    LOG(ERROR) << "Cannot create parameterized curve from spec: "
               << spec.DebugString();
    return NULL;
  }

  CHECK(DeserializeMatrix(
      spec.encoded_transform(), &(result->permanent_transform_)));
  return result;
}

bool ParameterizedCurve::ToSpec(RegisteredCurveSpec* spec) const {
  return ToSpecInternal(spec) &&
      SerializeMatrix(permanent_transform_,
                      spec->mutable_encoded_transform());
}

bool ParameterizedCurve::SerializeMatrix(const Matrix4d& mat, string* encoded) {
  if (mat == Matrix4d::Identity()) {
    *encoded = "";
    return true;
  }

  std::ostringstream strs;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      strs << mat(row, col) << "\t";
    }
  }
  *encoded = strs.str();

  return true;
}

bool ParameterizedCurve::DeserializeMatrix(const string& encoded, Matrix4d* mat) {
  if (encoded.empty()) {
    *mat = Matrix4d::Identity();
    return true;
  }

  std::istringstream istrs(encoded);
  std::ostringstream strs;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      istrs >> (*mat)(row, col);
    }
  }

  if (!istrs.good())
    LOG(ERROR) << "Error deserializing 4d matrix from: \""
               << encoded << "\"";

  return istrs.good();
}

double ParameterizedCurve::CapTimeToValidRange(double t) const {
  if (t > 1.0001 || t < 0) {
    LOG(FATAL) << "Capping time parameter to 0-1 range: " << t;
  }
  return fmax(0, fmin(t, 1.0));
}

bool ParameterizedCurve::is_closed() const {
  return mds::ApproxEqual(Position(0.0), Position(1.0));
}

void ParameterizedCurve::AddTransform(const Matrix4d& transform) {
  transform_ = transform * transform_;
}

void ParameterizedCurve::ClearTransforms() {
  transform_.setIdentity();
}

void ParameterizedCurve::BakeInTransforms() {
  permanent_transform_ = transform_ * permanent_transform_;
  ClearTransforms();
}

Vector3d ParameterizedCurve::Position(double t) const {
  t = CapTimeToValidRange(t);

  Vector4d position = Vector4d::Ones();
  position.head(3) = PositionInternal(t);

  position = transform_ * permanent_transform_ * position;
  position /= position[3];
  return position.head(3);
}

bool ParameterizedCurve::Discretize(vector<Vector3d>* positions) const {
  static const int min_pts_per_segment = 5;  // default: 5
  static const int min_pts_per_curve = 30;

  vector<double> pts;
  {
    set<double> tmp_pts;
    GetSalientPoints(&tmp_pts);
    CHECK_GE(tmp_pts.size(), 2);  // Assume that always included 0 and 1
    pts.insert(pts.begin(), tmp_pts.begin(), tmp_pts.end());
  }

  int num_segments = pts.size() - 1;
  int pts_per_segment = min_pts_per_segment;
  if (num_segments * pts_per_segment < min_pts_per_curve) {
    pts_per_segment =
        ceil(static_cast<double>(min_pts_per_curve) / num_segments);
  }

  for (int seg = 0; seg < num_segments; ++seg) {
    const double start_time = pts[seg];
    const double end_time = pts[seg + 1];
    const double increment = (end_time - start_time) / pts_per_segment;

    double cur_time = start_time;
    for (int pt = 0; pt < pts_per_segment; ++pt) {
      positions->push_back(Position(cur_time));
      cur_time += increment;
    }
  }

  // Add the very last point of the curve
  if (!is_closed()) {
    positions->push_back(Position(pts.back()));
  }

  return true;
}


// CIRCLE ----------------------------------------------------------------------

ParameterizedOval::ParameterizedOval(double radius1, double radius2)
    : r1_(radius1), r2_(radius2) {}

Vector3d ParameterizedOval::PositionInternal(double t) const {
  static const double pi = boost::math::constants::pi<long double>();

  // Convert t to radians
  double angle = -2 * pi * t;

  // Find position in the xz plane
  Vector3d pos;
  pos << r1_ * std::cos(angle), 0.0, r2_ * std::sin(angle);

  // YES, this is possible for buggy versions of glibc.
  // E.g. see:
  // http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=153022
  // if (!mds::ApproxEqual(pos.norm(), r_, 0.1)) {
  //   LOG(ERROR) << "Crazy Oval: "
  //              << "r: " << r_
  //              << "; pi: " << pi
  //              << "; cos: " << std::cos(angle)
  //              << "; sin: " << std::sin(angle)
  //              << "; t: " << t << "; angle: " << angle << "; vec = " << pos;
  // }

  return pos;
}

bool ParameterizedOval::ToSpecInternal(RegisteredCurveSpec* spec) const {
  CreateOvalSpec* ospec = spec->MutableExtension(CreateOvalSpec::spec);
  ospec->set_radius1(r1_);
  ospec->set_radius2(r2_);
  return true;
}

// CIRCLE ----------------------------------------------------------------------

ParameterizedCircle::ParameterizedCircle(double radius)
    : r_(radius) {}

Vector3d ParameterizedCircle::PositionInternal(double t) const {
  static const double pi = boost::math::constants::pi<long double>();

  // Convert t to radians
  double angle = -2 * pi * t;

  // Find position in the xz plane
  Vector3d pos;
  pos << r_ * std::cos(angle), 0.0, r_ * std::sin(angle);

  // YES, this is possible for buggy versions of glibc.
  // E.g. see:
  // http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=153022
  // if (!mds::ApproxEqual(pos.norm(), r_, 0.1)) {
  //   LOG(ERROR) << "Crazy Circle: "
  //              << "r: " << r_
  //              << "; pi: " << pi
  //              << "; cos: " << std::cos(angle)
  //              << "; sin: " << std::sin(angle)
  //              << "; t: " << t << "; angle: " << angle << "; vec = " << pos;
  // }

  return pos;
}

bool ParameterizedCircle::ToSpecInternal(RegisteredCurveSpec* spec) const {
  spec->MutableExtension(CreateCircleSpec::spec)->set_radius(r_);
  return true;
}

// YIN-YANG --------------------------------------------------------------------
ParameterizedYinYang::ParameterizedYinYang()
    : large_circle_(1.0),
      small_circle_((1 - std::acos(-1.0)/ 3.3) / 2.0) {}

void ParameterizedYinYang::GetSalientPoints(set<double>* pts) const {
  pts->insert(0.0);
  pts->insert(0.05);
  pts->insert(0.25);
  pts->insert(0.45);
  pts->insert(0.5);
  pts->insert(0.75);
  pts->insert(1.0);
}

bool ParameterizedYinYang::ToSpecInternal(RegisteredCurveSpec* spec) const {
  spec->MutableExtension(CreateYinYangCurveSpec::spec)->
      set_remove_sharp_point(false);
  return true;
}

Vector3d ParameterizedYinYang::PositionInternal(double t) const {
  if (t <= 0.5) {
    // Re-parameterized time so that:
    // t: 0.0  .............................. 0.5
    // x: 1    .............................. -1
    double x = 4 * (0.5 - t) - 1;
    double z = std::sin(-x * std::acos(-1.0)) * 0.7 / 3;
    return Vector3d(x, 0, z);
  } else {
    Vector3d pos = large_circle_.Position(t - 0.5);
    pos[0] = -pos[0];
    return pos;
  }
}

bool ParameterizedYinYang::Discretize(vector<Vector3d>* positions) const {
  positions->push_back(Vector3d(1, 0, 0));
  for (double t = 0.05; t <= 0.5; t += 0.05) {
    positions->push_back(Position(t));
  }
  for (double t = 0.55; t < 1.0; t+= 0.05) {
    positions->push_back(Position(t));
  }
  // TODO: something is not right, why is this needed?
  std::reverse(positions->begin(), positions->end());

  return true;
}

// SINE WAVE -------------------------------------------------------------------

ParameterizedSineWave::ParameterizedSineWave(const CreateSineWaveSpec& spec)
    : spec_(spec) {}

void ParameterizedSineWave::GetSalientPoints(set<double>* pts) const {
  pts->insert(0.0);
  pts->insert(0.45);
  pts->insert(0.5);
  pts->insert(0.95);
  pts->insert(1.0);
}

// Rough length along X-axis: ~2.5
// Length along Z-axis: thickness
// Want to allocate 2x many pts / length for the sine wave
// Say 10 is good for each sine wave
// Then each side should get 2 * thickness pts.
// In terms of time:
// 2 t + 2 w = 1; where
bool ParameterizedSineWave::Discretize(vector<Vector3d>* positions) const {
  static const double step = 0.05;

  positions->push_back(Vector3d(spec_.n_cycles(), 0, 0));
  for (double t = 0.05; t <= 0.45; t += (step / spec_.n_cycles())) {
    if (t + (step / spec_.n_cycles()) > 0.45) {
      t = 0.45;
    }
    positions->push_back(Position(t));
  }
  if (spec_.thickness() > 1.0) {
    positions->push_back(Position(0.475));
  }
  for (double t = 0.5; t <= 0.95; t+= (step / spec_.n_cycles())) {
    if (t + (step / spec_.n_cycles()) > 0.95) {
      t = 0.95;
    }
    positions->push_back(Position(t));
  }
  if (spec_.thickness() > 1.0) {
    positions->push_back(Position(0.975));
  }

  return true;

}

bool ParameterizedSineWave::ToSpecInternal(RegisteredCurveSpec* spec) const {
  spec->MutableExtension(CreateSineWaveSpec::spec)->CopyFrom(spec_);
  return true;
}

Vector3d ParameterizedSineWave::PositionInternal(double t) const {
  if (t <= 0.45) {
    // Re-parameterized time so that:
    // t: 0.0  .............................. 0.45
    // x: n_cycles    .............................. -n_cycles
    double x = spec_.n_cycles() * (2 / 0.45 * (0.45 - t) - 1);
    double z = std::sin(-x * std::acos(-1.0)) * spec_.amplitude();
    return Vector3d(x, 0, z);
  } else if (t <= 0.5) {
    double p = (0.45 - t) / 0.05;
    double z = p * spec_.thickness();
    return Vector3d(-spec_.n_cycles(), 0, z);
  } else if (t <= 0.95) {
    // Re-parameterize time so that:
    // t: 0.5  .............................. 0.95
    // x: -n_cycles   .............................. n_cycles
    double x = spec_.n_cycles() * (2 / 0.45 * (t - 0.95) + 1);
    double z = std::sin(-x * std::acos(-1.0)) * spec_.amplitude() -
               spec_.thickness();
    return Vector3d(x, 0, z);
  } else {
    double p = (t - 1.0) / 0.05;
    double z = p * spec_.thickness();
    return Vector3d(spec_.n_cycles(), 0, z);
  }
}


// POLYGON ---------------------------------------------------------------------

void ParameterizedPolygon::GetSalientPoints(set<double>* pts) const {
  const double interval_size = 1.0 / nsides_;

  pts->insert(0.0);
  for (int i = 0; i < nsides_; ++i) {
    pts->insert(*(--pts->end()) + interval_size);
  }
}

bool ParameterizedPolygon::Discretize(vector<Vector3d>* positions) const {
  set<double> pts;
  GetSalientPoints(&pts);
  CHECK_GE(pts.size(), 2);

  for (const double& t : pts) {
    Vector3d next = Position(t);
    positions->push_back(next);
  }

  positions->pop_back();  // always closed, so omit the last pt

  return true;
}

Vector3d ParameterizedPolygon::PositionInternal(double t) const {
  const double interval_size = 1.0 / nsides_;

  // Find previous and next vertices
  const int prev_vertex = floor(t / interval_size);
  const int next_vertex = ceil(t / interval_size);
  Vector3d prev_pos = circle_.Position(prev_vertex * interval_size);
  Vector3d next_pos = circle_.Position(next_vertex * interval_size);

  // interpolate between prev position and next position
  t -= interval_size * prev_vertex;
  t /= interval_size;
  return prev_pos * (1.0 - t) + next_pos * t;
}

bool ParameterizedPolygon::ToSpecInternal(RegisteredCurveSpec* spec) const {
  CreatePolygonSpec* poly_spec =
      spec->MutableExtension(CreatePolygonSpec::spec);
  poly_spec->set_radius(circle_.radius());
  poly_spec->set_n_sides(nsides_);
  return true;
}

// HERMITE CURVE ---------------------------------------------------------------
const Matrix4d HermiteCurve::hermite_ = HermiteCurve::HermiteMatrix();

HermiteCurve::HermiteCurve(const CreateHermiteSpec& spec) {
  spec_.CopyFrom(spec);

  if (spec_.tension() > 1.0) {
    spec_.set_tension(1.0);
  } else if (spec_.tension() < 0.0) {
    spec_.set_tension(0.0);
  }

  for (int i = 0; i < spec_.curve().control_pt_size(); ++i) {
    const PointVectorParam& cp = spec_.curve().control_pt(i);
    AddPoint(ToVector3d(cp.pt()),
             ToVector3d(cp.vec()),
             ToVector3d(cp.vec2()));
  }
}

void HermiteCurve::RemovePoint() {
  points_.pop_back();
  gradients_.pop_back();
  if (spec_.force_catmull_rom()) {
    ResetGradientsToCatmullRom();
  }
}

void HermiteCurve::AddPoint(const Vector3d& position) {
  AddPoint(position, Vector3d(0,0,0), Vector3d(0,0,0));
}

void HermiteCurve::AddPoint(const Vector3d& position,
                            const Vector3d& gradient,
                            const Vector3d& second_gradient) {
  if (!points_.empty() &&
      mds::ApproxEqual(points_.back(), position)) {
    LOG(ERROR) << "Cannot add second point with the same position. ";
    return;
  }

  points_.push_back(position);
  gradients_.push_back(vector<Vector3d>());
  gradients_.back().push_back(gradient);
  gradients_.back().push_back(second_gradient);

  // TODO: call in a more appropriate place
  if (spec_.force_catmull_rom()) {
    ResetGradientsToCatmullRom();
  }
}

void HermiteCurve::ResetPointToCatmullRom(const int id, const double tension) {
  // TODO: fix the hackiness
  gradients_[id].clear();
  bool closed = mds::ApproxEqual(points_[0], points_.back());
  Vector3d vec = Vector3d::Zero();

  if (id == 0) {
    if (closed) {
      const Vector3d& prev = points_[points_.size() - 2];
      const Vector3d& next = points_[1];
      vec = (1 - tension) * (next - prev) / 2.0;
    }
  } else if (id == points_.size() - 1) {
    if (closed) {
      const Vector3d& prev = points_[points_.size() - 2];
      const Vector3d& next = points_[1];
      vec = (1 - tension) * (next - prev) / 2.0;
    }
  } else {
    const Vector3d& prev = points_[(id - 1) % points_.size()];
    const Vector3d& next = points_[(id + 1) % points_.size()];
    vec = (1 - tension) * (next - prev) / 2.0;
  }
  gradients_[id].push_back(vec);
  gradients_[id].push_back(vec);
}

void HermiteCurve::ResetGradientsToCatmullRom() {
  if (points_.size() < 3) return;

  gradients_.clear();

  bool closed = mds::ApproxEqual(points_[0], points_.back());
  gradients_.push_back(vector<Vector3d>());
  if (!closed) {
    gradients_.back().push_back(Vector3d::Zero());
    gradients_.back().push_back(Vector3d::Zero());
  } else {
    const Vector3d& prev = points_[points_.size() - 2];
    const Vector3d& next = points_[1];
    Vector3d vec = (1 - spec_.tension()) * (next - prev) / 2.0;
    gradients_.back().push_back(vec);
    gradients_.back().push_back(vec);
  }

  for (int i = 1; i < points_.size() - 1; ++i) {
    const Vector3d& prev = points_[(i - 1) % points_.size()];
    const Vector3d& next = points_[(i + 1) % points_.size()];
    Vector3d vec = (1 - spec_.tension()) * (next - prev) / 2.0;
    gradients_.push_back(vector<Vector3d>());
    gradients_.back().push_back(vec);
    gradients_.back().push_back(vec);
  }
  gradients_.push_back(vector<Vector3d>());
  gradients_.back().push_back(gradients_[0][0]);
  gradients_.back().push_back(gradients_[0][1]);
}

void HermiteCurve::force_catmull_rom(const double tension) {
  spec_.set_tension(fmax(0, fmin(1, tension)));
  spec_.set_force_catmull_rom(true);
  ResetGradientsToCatmullRom();
}

void HermiteCurve::Reset() {
  points_.clear();
  gradients_.clear();
}

// pt0 weight:   H0(t) = 2t^3 - 3t^2 + 1
// grad0 weight: H1(t) = t^3 - 2t^2 + t
// pt1 weight:   H3(t) = -2t^3 + 3t^2
// grad1 weight: H2(t) = t^3 - t^2
Matrix4d HermiteCurve::HermiteMatrix() {
  Matrix4d hermite;
  hermite <<
      2, -3, 0, 1,  // H0
      1, -2, 1, 0,  // H1
      -2, 3, 0, 0,  // H3
      1, -1, 0, 0;  // H2
  return hermite;
}

Vector4d HermiteCurve::TimeVector(double t) const {
  Vector4d u_vec;
  u_vec << t*t*t, t*t, t, 1;
  return u_vec;
}

MatrixXd HermiteCurve::ControlsMatrix(int index) const {
  CHECK_GE(index, 0);
  CHECK_LT(index, points_.size() - 1);
  CHECK_LT(index, gradients_.size() - 1);
  CHECK_EQ(2, gradients_[index].size());
  CHECK(!gradients_[index+1].empty());

  MatrixXd matrix(3, 4);
  matrix.col(0) = points_[index];
  matrix.col(1) = gradients_[index].back();
  matrix.col(2) = points_[index+1];
  matrix.col(3) = gradients_[index+1].front();
  return matrix;
}

void HermiteCurve::GetSalientPoints(set<double>* pts) const {
  double interval_size = 1.0 / (points_.size() - 1);

  pts->insert(0.0);
  for (int i = 0; i < points_.size() - 1; ++i) {
    pts->insert(*(--pts->end()) + interval_size);
  }
}

bool HermiteCurve::Discretize(vector<Vector3d>* positions) const {
  double interval_size = 1.0 / (points_.size() - 1);

  double cur_time = 0.0;
  for (int i = 0; i < points_.size() - 1; ++i) {
    positions->push_back(Position(cur_time));
    cur_time += interval_size * 0.2;

    for (int j = 0; j < 6; ++j) {
      positions->push_back(Position(cur_time));
      cur_time += interval_size * 0.1;
    }

    positions->push_back(Position(cur_time));
    cur_time += interval_size * 0.2;
  }

  if (!is_closed()) {
    positions->push_back(Position(1.0));
  }
  return true;
}

Vector3d HermiteCurve::PositionInternal(double t) const {
  double interval_size = 1.0 / (points_.size() - 1);
  int index = floor(t / interval_size);

  // In case t = 1.0, want to get the control points matrix indexed
  // by the next to last point.
  if (index >= points_.size() - 1) {
    index = points_.size() - 2;
    t = 1.0;
  } else {
    // Next, map this t to a t' that is between two adjacent control pts
    t -= interval_size * index;
    t /= interval_size;  // renormalize
  }
  return ControlsMatrix(index) * hermite_ * TimeVector(t);
}

bool HermiteCurve::ToSpecInternal(RegisteredCurveSpec* spec) const {
  spec->MutableExtension(CreateHermiteSpec::spec)->CopyFrom(spec_);
  CurveParams* curve_params =
      spec->MutableExtension(CreateHermiteSpec::spec)->mutable_curve();
  curve_params->Clear();

  for (int i = 0; i < points_.size(); ++i) {
    PointVectorParam* pt_param = curve_params->add_control_pt();
    pt_param->mutable_pt()->CopyFrom(ToVectorParam(points_[i]));

    if (gradients_[i].size() > 0) {
      pt_param->mutable_vec()->CopyFrom(ToVectorParam(gradients_[i][0]));
    }
    if (gradients_[i].size() > 1) {
      pt_param->mutable_vec2()->CopyFrom(ToVectorParam(gradients_[i][1]));
    }
  }
  return true;
}

namespace {
bool FindNonzeroPixel(const QImage& bitmap, int* nzx, int* nzy) {
  for (int x = 0; x < bitmap.width(); ++x) {
    for (int y = 0; y < bitmap.height(); ++y) {
      QRgb p = bitmap.pixel(x, y);
      if (qGray(p) != 0) {
        *nzx = x;
        *nzy = y;
        return true;
      }
    }
  }
  return false;
}
}  // namespace

ImageCurve::ImageCurve(const CreateImageCurveSpec& spec) {
  spec_.CopyFrom(spec);
  // TODO: remove QT dependency
  QImage bitmap(QString(spec.image_filename().c_str()));
  VLOG(1) << "Image: " << bitmap.width() << " x " << bitmap.height();

  int x_start(0), y_start(0);
  CHECK(FindNonzeroPixel(bitmap, &x_start, &y_start));

  int x = x_start, y = y_start;
  vector<std::pair<int, int> > pts;
  pts.push_back(std::make_pair(x, y));
  while (1) {
    // Check neighbors
    bool broken = false;
    for (int xn = fmax(0, x - 1); xn <= fmin(bitmap.width() - 1, x + 1); ++xn) {
      for (int yn = fmax(0, y - 1); yn <= fmin(bitmap.height() - 1, y + 1); ++yn) {
        if (qGray(bitmap.pixel(xn, yn)) != 0 &&
            (xn != x || yn != y) &&
            find(pts.begin(), pts.end(), std::make_pair(xn, yn)) == pts.end()) {
          VLOG(1) << "Adding pt " << xn << ", " << yn;
          pts.push_back(std::make_pair(xn, yn));
          curve_.AddPoint(Vector3d(xn - bitmap.width() / 2.0 , 0, yn - bitmap.height() / 2.0),
                          Vector3d(0,0,0), Vector3d(0,0,0));
          x = xn;
          y = yn;
          broken = true;
          break;
        }
        if (broken) break;
      }
    }

    if ((x == x_start && y == y_start) || !broken) break;
  }

}

bool ImageCurve::ToSpecInternal(RegisteredCurveSpec* spec) const {
  spec->MutableExtension(CreateImageCurveSpec::spec)->CopyFrom(spec_);
  return true;
}

}  // namespace mit_plato
