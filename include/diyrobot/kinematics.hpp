#pragma once

#include <array>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

namespace diyrobot {

class KinematicsError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
class PathUnsafeError : public KinematicsError {
 public:
  using KinematicsError::KinematicsError;
};

struct JointCalibration {
  double mid{}, minimum{}, maximum{};
};
using JointMap = std::map<std::string, double>;
using CalibrationMap = std::map<std::string, JointCalibration>;

struct KinematicsConfig {
  double shoulder_x{0.0388}, shoulder_y{0.0}, shoulder_h{0.0624};
  double link1{0.1776}, link2{0.1350}, wrist_to_tip{0.0980};
  JointMap sign{{"shoulder_pan", 1}, {"shoulder_lift", 1}, {"elbow_flex", 1}, {"wrist_flex", 1}};
  JointMap offset_deg{
      {"shoulder_pan", 0}, {"shoulder_lift", 19.3}, {"elbow_flex", 68.5}, {"wrist_flex", 5.0}};
  bool elbow_up{true};
  std::optional<double> safe_z_min;
};

class SO101Kinematics {
 public:
  SO101Kinematics(KinematicsConfig config, CalibrationMap calibration = {});
  int deg_to_code(const std::string& joint, double degrees) const;
  double code_to_deg(const std::string& joint, int code) const;
  double clamp_joint(const std::string& joint, double degrees) const;
  std::array<double, 3> fk(const JointMap& joints) const;
  JointMap ik_vertical(double x, double y, double z) const;
  void check_path_safe(const JointMap& current, const JointMap& target, std::optional<double> z_min,
                       unsigned samples = 40) const;
  JointMap home_pose() const;
  const std::map<std::string, std::pair<double, double>>& limits_deg() const {
    return limits_deg_;
  }

 private:
  double phi(const char* joint, double degrees) const;
  KinematicsConfig config_;
  CalibrationMap calibration_;
  std::map<std::string, std::pair<double, double>> limits_deg_;
};

}  // namespace diyrobot
