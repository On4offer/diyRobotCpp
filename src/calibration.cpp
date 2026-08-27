#include "diyrobot/calibration.hpp"

#include <algorithm>
#include <cmath>

namespace diyrobot {
namespace {
template <std::size_t N>
std::array<double, N> solve_linear(std::array<std::array<double, N>, N> a,
                                   std::array<double, N> b) {
  for (std::size_t col = 0; col < N; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < N; ++row) {
      if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(a[pivot][col]) < 1e-12) {
      throw CalibrationError("degenerate calibration points");
    }
    std::swap(a[pivot], a[col]);
    std::swap(b[pivot], b[col]);
    const double divisor = a[col][col];
    for (std::size_t j = col; j < N; ++j) {
      a[col][j] /= divisor;
    }
    b[col] /= divisor;
    for (std::size_t row = 0; row < N; ++row) {
      if (row != col) {
        const double factor = a[row][col];
        for (std::size_t j = col; j < N; ++j) {
          a[row][j] -= factor * a[col][j];
        }
        b[row] -= factor * b[col];
      }
    }
  }
  return b;
}
Matrix3 inverse(const Matrix3& m) {
  const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (std::abs(det) < 1e-15) {
    throw CalibrationError("homography is singular");
  }
  Matrix3 out{{{{m[1][1] * m[2][2] - m[1][2] * m[2][1], m[0][2] * m[2][1] - m[0][1] * m[2][2],
                 m[0][1] * m[1][2] - m[0][2] * m[1][1]}},
               {{m[1][2] * m[2][0] - m[1][0] * m[2][2], m[0][0] * m[2][2] - m[0][2] * m[2][0],
                 m[0][2] * m[1][0] - m[0][0] * m[1][2]}},
               {{m[1][0] * m[2][1] - m[1][1] * m[2][0], m[0][1] * m[2][0] - m[0][0] * m[2][1],
                 m[0][0] * m[1][1] - m[0][1] * m[1][0]}}}};
  for (auto& row : out) {
    for (auto& value : row) {
      value /= det;
    }
  }
  return out;
}
Point2 transform(const Matrix3& h, double x, double y) {
  const double w = h[2][0] * x + h[2][1] * y + h[2][2];
  if (std::abs(w) < 1e-15) {
    throw CalibrationError("homogeneous point at infinity");
  }
  return {(h[0][0] * x + h[0][1] * y + h[0][2]) / w, (h[1][0] * x + h[1][1] * y + h[1][2]) / w};
}
}  // namespace

void TableCalibration::add_point(Point2 pixel, Point2 base) {
  pixels_.push_back(pixel);
  bases_.push_back(base);
  solved_ = false;
}
ReprojectionError TableCalibration::solve() {
  if (pixels_.size() < 4 || pixels_.size() != bases_.size()) {
    throw CalibrationError("at least four point pairs are required");
  }
  std::array<std::array<double, 8>, 8> ata{};
  std::array<double, 8> atb{};
  auto accumulate = [&](const std::array<double, 8>& row, double value) {
    for (std::size_t i = 0; i < 8; ++i) {
      atb[i] += row[i] * value;
      for (std::size_t j = 0; j < 8; ++j) {
        ata[i][j] += row[i] * row[j];
      }
    }
  };
  for (std::size_t i = 0; i < pixels_.size(); ++i) {
    const auto [u, v] = pixels_[i];
    const auto [x, y] = bases_[i];
    accumulate({u, v, 1, 0, 0, 0, -x * u, -x * v}, x);
    accumulate({0, 0, 0, u, v, 1, -y * u, -y * v}, y);
  }
  const auto h = solve_linear<8>(ata, atb);
  matrix_ = {{{{h[0], h[1], h[2]}}, {{h[3], h[4], h[5]}}, {{h[6], h[7], 1.0}}}};
  solved_ = true;
  return reprojection_error();
}
Point2 TableCalibration::pixel_to_base(double u, double v) const {
  if (!solved_) {
    throw CalibrationError("calibration not solved");
  }
  return transform(matrix_, u, v);
}
Point2 TableCalibration::base_to_pixel(double x, double y) const {
  if (!solved_) {
    throw CalibrationError("calibration not solved");
  }
  return transform(inverse(matrix_), x, y);
}
ReprojectionError TableCalibration::reprojection_error() const {
  if (!solved_ || pixels_.empty()) {
    throw CalibrationError("calibration not solved");
  }
  double px2 = 0, mm2 = 0;
  for (std::size_t i = 0; i < pixels_.size(); ++i) {
    const auto p = pixel_to_base(pixels_[i][0], pixels_[i][1]);
    const auto q = base_to_pixel(bases_[i][0], bases_[i][1]);
    mm2 += std::pow(p[0] - bases_[i][0], 2) + std::pow(p[1] - bases_[i][1], 2);
    px2 += std::pow(q[0] - pixels_[i][0], 2) + std::pow(q[1] - pixels_[i][1], 2);
  }
  const double count = static_cast<double>(pixels_.size());
  return {std::sqrt(px2 / count), std::sqrt(mm2 / count) * 1000.0};
}
const Matrix3& TableCalibration::homography() const {
  if (!solved_) {
    throw CalibrationError("calibration not solved");
  }
  return matrix_;
}
void TableCalibration::set_homography(Matrix3 matrix) {
  matrix_ = matrix;
  solved_ = true;
}
std::vector<Point2> grid_xy(double xs, double xe, unsigned nx, double ys, double ye, unsigned ny) {
  if (nx < 1 || ny < 1) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  std::vector<Point2> out;
  for (unsigned i = 0; i < nx; ++i) {
    for (unsigned j = 0; j < ny; ++j) {
      out.push_back({nx == 1 ? xs : xs + (xe - xs) * i / (nx - 1),
                     ny == 1 ? ys : ys + (ye - ys) * j / (ny - 1)});
    }
  }
  return out;
}

}  // namespace diyrobot
