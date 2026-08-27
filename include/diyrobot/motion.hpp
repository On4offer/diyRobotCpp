#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace diyrobot {

class TrapezoidalProfile {
 public:
  TrapezoidalProfile(double q0, double qf, double v_max, double a_max);
  double position(double t) const;
  double velocity(double t) const;
  double duration() const {
    return duration_;
  }
  double acceleration_time() const {
    return t_accel_;
  }
  double cruise_time() const {
    return t_cruise_;
  }
  double peak_velocity() const {
    return v_peak_;
  }

 private:
  double q0_{}, qf_{}, v_max_{}, a_max_{}, v_peak_{}, t_accel_{}, t_cruise_{}, duration_{},
      accel_distance_{};
};

class LinearProfile {
 public:
  LinearProfile(double q0, double qf, double duration);
  double position(double t) const;

 private:
  double q0_{}, qf_{}, duration_{};
};

class EaseProfile {
 public:
  EaseProfile(double q0, double qf, double duration);
  double position(double t) const;

 private:
  double q0_{}, qf_{}, duration_{};
};

double ease_in_out_sine(double t);
double clamp_relative(double goal, double present, double max_diff);

class ServoIO {
 public:
  virtual ~ServoIO() = default;
  virtual int present_position(std::uint8_t id) = 0;
  virtual std::pair<int, int> position_limits(std::uint8_t id) = 0;
  virtual void goal_position(std::uint8_t id, int value) = 0;
  virtual void torque(std::uint8_t id, bool enabled) = 0;
};

struct MotionSample {
  double time{};
  std::uint8_t servo_id{};
  int goal{};
  int present{};
};
struct MotionResult {
  double duration{};
  std::map<std::uint8_t, int> start, end;
  std::vector<MotionSample> samples;
};

class ArmController {
 public:
  ArmController(ServoIO& io, std::vector<std::uint8_t> ids, unsigned fps = 50,
                double max_step = 0.0);
  MotionResult move_to(const std::map<std::uint8_t, double>& targets,
                       const std::string& profile = "trapezoid", double v_max = 200.0,
                       double a_max = 400.0, double duration = 0.0, bool real_time = true);
  void emergency_stop();

 private:
  ServoIO& io_;
  std::vector<std::uint8_t> ids_;
  unsigned fps_;
  double max_step_;
};

}  // namespace diyrobot
