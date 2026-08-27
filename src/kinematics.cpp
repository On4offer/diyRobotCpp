#include "diyrobot/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace diyrobot {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double resolution = 4095.0;
}  // namespace

SO101Kinematics::SO101Kinematics(KinematicsConfig config, CalibrationMap calibration)
    : config_(std::move(config)), calibration_(std::move(calibration)) {
  for (const auto& [name, cal] : calibration_) {
    const double a = (cal.minimum - cal.mid) * 360.0 / resolution;
    const double b = (cal.maximum - cal.mid) * 360.0 / resolution;
    limits_deg_[name] = {std::min(a, b), std::max(a, b)};
  }
}
int SO101Kinematics::deg_to_code(const std::string& joint, double degrees) const {
  const auto it = calibration_.find(joint);
  if (it == calibration_.end()) {
    throw KinematicsError("missing joint calibration");
  }
  return static_cast<int>(std::lround(degrees * resolution / 360.0 + it->second.mid));
}
double SO101Kinematics::code_to_deg(const std::string& joint, int code) const {
  const auto it = calibration_.find(joint);
  if (it == calibration_.end()) {
    throw KinematicsError("missing joint calibration");
  }
  return (code - it->second.mid) * 360.0 / resolution;
}
double SO101Kinematics::clamp_joint(const std::string& joint, double degrees) const {
  const auto it = limits_deg_.find(joint);
  return it == limits_deg_.end() ? degrees
                                 : std::clamp(degrees, it->second.first, it->second.second);
}
double SO101Kinematics::phi(const char* joint, double degrees) const {
  return (config_.sign.at(joint) * degrees + config_.offset_deg.at(joint)) * pi / 180.0;
}
std::array<double, 3> SO101Kinematics::fk(const JointMap& joints) const {
  const double p2 = phi("shoulder_lift", joints.at("shoulder_lift"));
  const double p3 = phi("elbow_flex", joints.at("elbow_flex"));
  const double p4 = phi("wrist_flex", joints.at("wrist_flex"));
  const double rw = config_.link1 * std::sin(p2) + config_.link2 * std::sin(p2 + p3);
  const double zw = config_.link1 * std::cos(p2) + config_.link2 * std::cos(p2 + p3);
  const double tip = p2 + p3 + p4;
  const double rt = rw + config_.wrist_to_tip * std::sin(tip);
  const double zt = zw + config_.wrist_to_tip * std::cos(tip);
  const double p1 = phi("shoulder_pan", joints.at("shoulder_pan"));
  return {config_.shoulder_x + rt * std::cos(p1), config_.shoulder_y + rt * std::sin(p1),
          config_.shoulder_h + zt};
}
JointMap SO101Kinematics::ik_vertical(double x, double y, double z) const {
  if (config_.safe_z_min && z < *config_.safe_z_min) {
    throw KinematicsError("target is below safe table height");
  }
  const double rx = x - config_.shoulder_x, ry = y - config_.shoulder_y;
  const double r = std::hypot(rx, ry);
  if (r > config_.link1 + config_.link2 + 1e-6) {
    throw KinematicsError("target exceeds horizontal reach");
  }
  const double p1 = std::atan2(ry, rx);
  const double pan =
      (p1 * 180.0 / pi - config_.offset_deg.at("shoulder_pan")) / config_.sign.at("shoulder_pan");
  const double rw = r, zw = z - config_.shoulder_h + config_.wrist_to_tip;
  const double distance = std::hypot(rw, zw);
  if (distance < 1e-4 || distance < std::abs(config_.link1 - config_.link2) - 1e-6 ||
      distance > config_.link1 + config_.link2 + 1e-6) {
    throw KinematicsError("wrist target is unreachable");
  }
  const double cosa = std::clamp(
      (distance * distance - config_.link1 * config_.link1 - config_.link2 * config_.link2) /
          (2.0 * config_.link1 * config_.link2),
      -1.0, 1.0);
  const double p3 = (config_.elbow_up ? 1.0 : -1.0) * std::acos(cosa);
  const double beta = std::atan2(rw, zw);
  const double gamma =
      std::atan2(config_.link2 * std::sin(p3), config_.link1 + config_.link2 * std::cos(p3));
  const double p2 = beta - gamma, p4 = pi - p2 - p3;
  JointMap raw{{"shoulder_pan", pan},
               {"shoulder_lift", (p2 * 180.0 / pi - config_.offset_deg.at("shoulder_lift")) /
                                     config_.sign.at("shoulder_lift")},
               {"elbow_flex", (p3 * 180.0 / pi - config_.offset_deg.at("elbow_flex")) /
                                  config_.sign.at("elbow_flex")},
               {"wrist_flex", (p4 * 180.0 / pi - config_.offset_deg.at("wrist_flex")) /
                                  config_.sign.at("wrist_flex")}};
  JointMap out;
  for (const auto& [joint, value] : raw) {
    out[joint] = clamp_joint(joint, value);
    if (std::abs(out[joint] - value) > 1e-6) {
      throw KinematicsError("IK result violates joint limits");
    }
  }
  return out;
}
void SO101Kinematics::check_path_safe(const JointMap& current, const JointMap& target,
                                      std::optional<double> z_min, unsigned samples) const {
  if (!z_min) {
    return;
  }
  if (samples == 0) {
    throw std::invalid_argument("samples must be positive");
  }
  for (unsigned i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    JointMap pose = current;
    for (const auto& [joint, value] : target) {
      const double start = current.count(joint) ? current.at(joint) : value;
      pose[joint] = start + (value - start) * t;
    }
    if (!pose.count("wrist_roll")) {
      pose["wrist_roll"] = 0.0;
    }
    if (fk(pose)[2] < *z_min - 1e-3) {
      std::ostringstream msg;
      msg << "unsafe path at t=" << t << ": tip below z_min";
      throw PathUnsafeError(msg.str());
    }
  }
}
JointMap SO101Kinematics::home_pose() const {
  JointMap result;
  for (const auto& [joint, unused] : calibration_) {
    (void)unused;
    result[joint] = 0.0;
  }
  return result;
}

}  // namespace diyrobot
