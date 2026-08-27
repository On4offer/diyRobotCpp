#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace diyrobot {

struct Bgr {
  std::uint8_t b{}, g{}, r{};
};
class Image {
 public:
  Image(unsigned width, unsigned height, Bgr fill = {});
  unsigned width() const {
    return width_;
  }
  unsigned height() const {
    return height_;
  }
  Bgr& at(unsigned x, unsigned y);
  const Bgr& at(unsigned x, unsigned y) const;
  void fill_rect(unsigned x, unsigned y, unsigned width, unsigned height, Bgr color);

 private:
  unsigned width_{}, height_{};
  std::vector<Bgr> pixels_;
};

struct HsvRange {
  int h_low{}, h_high{179}, s_low{}, s_high{255}, v_low{}, v_high{255};
};
struct Target {
  int x{}, y{};
  double area{};
  int left{}, top{}, width{}, height{};
  std::optional<double> long_axis_degrees;
};

class ColorTargetDetector {
 public:
  ColorTargetDetector(HsvRange range, double minimum_area = 600, double maximum_area = 120000,
                      unsigned max_targets = 1);
  std::vector<std::uint8_t> mask(const Image& frame) const;
  std::vector<Target> detect(const Image& frame) const;
  std::optional<Target> detect_one(const Image& frame) const;

 private:
  HsvRange range_;
  double minimum_area_, maximum_area_;
  unsigned max_targets_;
};

}  // namespace diyrobot
