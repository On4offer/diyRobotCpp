#include "diyrobot/vision.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace diyrobot {
Image::Image(unsigned width, unsigned height, Bgr fill)
    : width_(width), height_(height), pixels_(static_cast<std::size_t>(width) * height, fill) {
  if (width == 0 || height == 0) {
    throw std::invalid_argument("image dimensions must be positive");
  }
}
Bgr& Image::at(unsigned x, unsigned y) {
  return pixels_.at(static_cast<std::size_t>(y) * width_ + x);
}
const Bgr& Image::at(unsigned x, unsigned y) const {
  return pixels_.at(static_cast<std::size_t>(y) * width_ + x);
}
void Image::fill_rect(unsigned x, unsigned y, unsigned w, unsigned h, Bgr c) {
  for (unsigned py = y; py < std::min(y + h, height_); ++py) {
    for (unsigned px = x; px < std::min(x + w, width_); ++px) {
      at(px, py) = c;
    }
  }
}
namespace {
std::array<int, 3> bgr_to_hsv(Bgr p) {
  const double b = p.b / 255.0, g = p.g / 255.0, r = p.r / 255.0;
  const double hi = std::max({r, g, b}), lo = std::min({r, g, b}), delta = hi - lo;
  double hue = 0;
  if (delta > 0) {
    if (hi == r) {
      hue = 60.0 * std::fmod((g - b) / delta, 6.0);
    } else if (hi == g) {
      hue = 60.0 * ((b - r) / delta + 2.0);
    } else {
      hue = 60.0 * ((r - g) / delta + 4.0);
    }
  }
  if (hue < 0) {
    hue += 360.0;
  }
  return {static_cast<int>(std::lround(hue / 2.0)),
          static_cast<int>(std::lround((hi == 0 ? 0 : delta / hi) * 255.0)),
          static_cast<int>(std::lround(hi * 255.0))};
}
}  // namespace
ColorTargetDetector::ColorTargetDetector(HsvRange range, double minimum, double maximum,
                                         unsigned count)
    : range_(range), minimum_area_(minimum), maximum_area_(maximum), max_targets_(count) {
  if (minimum < 0 || maximum < minimum || count == 0) {
    throw std::invalid_argument("invalid detector limits");
  }
}
std::vector<std::uint8_t> ColorTargetDetector::mask(const Image& frame) const {
  std::vector<std::uint8_t> out(static_cast<std::size_t>(frame.width()) * frame.height());
  for (unsigned y = 0; y < frame.height(); ++y) {
    for (unsigned x = 0; x < frame.width(); ++x) {
      const auto hsv = bgr_to_hsv(frame.at(x, y));
      const bool hue = range_.h_low <= range_.h_high
                           ? hsv[0] >= range_.h_low && hsv[0] <= range_.h_high
                           : hsv[0] >= range_.h_low || hsv[0] <= range_.h_high;
      out[static_cast<std::size_t>(y) * frame.width() + x] =
          static_cast<std::uint8_t>(hue && hsv[1] >= range_.s_low && hsv[1] <= range_.s_high &&
                                    hsv[2] >= range_.v_low && hsv[2] <= range_.v_high);
    }
  }
  return out;
}
std::vector<Target> ColorTargetDetector::detect(const Image& frame) const {
  auto binary = mask(frame);
  std::vector<std::uint8_t> seen(binary.size());
  std::vector<Target> targets;
  const int width = static_cast<int>(frame.width()), height = static_cast<int>(frame.height());
  for (int sy = 0; sy < height; ++sy) {
    for (int sx = 0; sx < width; ++sx) {
      const auto start = static_cast<std::size_t>(sy) * frame.width() + sx;
      if (!binary[start] || seen[start]) {
        continue;
      }
      std::queue<std::pair<int, int>> q;
      q.push({sx, sy});
      seen[start] = 1;
      double sumx = 0, sumy = 0, sumxx = 0, sumyy = 0, sumxy = 0, count = 0;
      int left = sx, right = sx, top = sy, bottom = sy;
      while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        sumx += x;
        sumy += y;
        sumxx += x * x;
        sumyy += y * y;
        sumxy += x * y;
        ++count;
        left = std::min(left, x);
        right = std::max(right, x);
        top = std::min(top, y);
        bottom = std::max(bottom, y);
        constexpr int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
          const int nx = x + dx[k], ny = y + dy[k];
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
            continue;
          }
          const auto idx = static_cast<std::size_t>(ny) * frame.width() + nx;
          if (binary[idx] && !seen[idx]) {
            seen[idx] = 1;
            q.push({nx, ny});
          }
        }
      }
      if (count < minimum_area_ || count > maximum_area_) {
        continue;
      }
      const double mx = sumx / count, my = sumy / count, cxx = sumxx / count - mx * mx,
                   cyy = sumyy / count - my * my, cxy = sumxy / count - mx * my;
      std::optional<double> angle;
      const double boxw = right - left + 1, boxh = bottom - top + 1,
                   aspect = std::max(boxw, boxh) / std::max(1.0, std::min(boxw, boxh));
      if (count >= 400 && aspect >= 1.2) {
        angle = 0.5 * std::atan2(2 * cxy, cxx - cyy) * 180.0 / 3.14159265358979323846;
      }
      targets.push_back({static_cast<int>(std::lround(mx)), static_cast<int>(std::lround(my)),
                         count, left, top, right - left + 1, bottom - top + 1, angle});
    }
  }
  std::sort(targets.begin(), targets.end(),
            [](const Target& a, const Target& b) { return a.area > b.area; });
  if (targets.size() > max_targets_) {
    targets.resize(max_targets_);
  }
  return targets;
}
std::optional<Target> ColorTargetDetector::detect_one(const Image& frame) const {
  auto all = detect(frame);
  return all.empty() ? std::nullopt : std::optional<Target>(all.front());
}
}  // namespace diyrobot
