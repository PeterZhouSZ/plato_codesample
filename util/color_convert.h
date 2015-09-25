#ifndef _UTIL_COLOR_CONVERT_H__
#define _UTIL_COLOR_CONVERT_H__

namespace mds_util {

struct Rgb {
  Rgb() : r(1.0), g(1.0), b(1.0) {}
  Rgb(double _r, double _g, double _b)
      : r(_r), g(_g), b(_b) {}

    double r;       // percent
    double g;       // percent
    double b;       // percent
};

struct Hsv {
  Hsv() : h(360), s(1.0), v(1.0) {}
  Hsv(double _h, double _s, double _v)
      : h(_h), s(_s), v(_v) {}

    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
};

Hsv rgb2hsv(Rgb in);
Rgb hsv2rgb(Hsv in);

}  // namespace mds

#endif  // _UTIL_COLOR_CONVERT_H__
