#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace diyrobot {

class CalibrationError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
using Point2 = std::array<double, 2>;
using Matrix3 = std::array<std::array<double, 3>, 3>;
struct ReprojectionError {
  double rms_pixels{}, rms_millimeters{};
};

class TableCalibration {
 public:
  void add_point(Point2 pixel, Point2 base);
  std::size_t size() const {
    return pixels_.size();
  }
  ReprojectionError solve();
  Point2 pixel_to_base(double u, double v) const;
  Point2 base_to_pixel(double x, double y) const;
  ReprojectionError reprojection_error() const;
  const Matrix3& homography() const;
  void set_homography(Matrix3 matrix);
  const std::vector<Point2>& pixel_points() const {
    return pixels_;
  }
  const std::vector<Point2>& base_points() const {
    return bases_;
  }

 private:
  std::vector<Point2> pixels_, bases_;
  Matrix3 matrix_{};
  bool solved_{false};
};

std::vector<Point2> grid_xy(double x_start, double x_stop, unsigned nx, double y_start,
                            double y_stop, unsigned ny);

}  // namespace diyrobot
