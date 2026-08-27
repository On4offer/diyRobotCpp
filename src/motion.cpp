#include "diyrobot/motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace diyrobot {

TrapezoidalProfile::TrapezoidalProfile(double q0, double qf, double v_max, double a_max)
    : q0_(q0), qf_(qf), v_max_(v_max), a_max_(a_max) {
  if (v_max <= 0.0 || a_max <= 0.0) {
    throw std::invalid_argument("v_max and a_max must be positive");
  }
  const double distance = std::abs(qf - q0);
  const double direction = qf >= q0 ? 1.0 : -1.0;
  const double full_accel_distance = v_max * v_max / (2.0 * a_max);
  if (distance <= 2.0 * full_accel_distance) {
    const double peak = std::sqrt(a_max * distance);
    v_peak_ = direction * peak;
    t_accel_ = peak / a_max;
    t_cruise_ = 0.0;
  } else {
    v_peak_ = direction * v_max;
    t_accel_ = v_max / a_max;
    t_cruise_ = (distance - 2.0 * full_accel_distance) / v_max;
  }
  duration_ = 2.0 * t_accel_ + t_cruise_;
  accel_distance_ = direction * 0.5 * a_max * t_accel_ * t_accel_;
}
double TrapezoidalProfile::position(double t) const {
  if (t <= 0.0 || duration_ == 0.0) {
    return q0_;
  }
  if (t >= duration_) {
    return qf_;
  }
  const double acceleration = v_peak_ / t_accel_;
  if (t < t_accel_) {
    return q0_ + 0.5 * acceleration * t * t;
  }
  if (t < t_accel_ + t_cruise_) {
    return q0_ + accel_distance_ + v_peak_ * (t - t_accel_);
  }
  const double remaining = duration_ - t;
  return qf_ - 0.5 * acceleration * remaining * remaining;
}
double TrapezoidalProfile::velocity(double t) const {
  if (t <= 0.0 || t >= duration_ || duration_ == 0.0) {
    return 0.0;
  }
  const double acceleration = v_peak_ / t_accel_;
  if (t < t_accel_) {
    return acceleration * t;
  }
  if (t < t_accel_ + t_cruise_) {
    return v_peak_;
  }
  return acceleration * (duration_ - t);
}
LinearProfile::LinearProfile(double q0, double qf, double duration)
    : q0_(q0), qf_(qf), duration_(duration) {
  if (duration <= 0.0) {
    throw std::invalid_argument("duration must be positive");
  }
}
double LinearProfile::position(double t) const {
  t = std::clamp(t, 0.0, duration_);
  return q0_ + (qf_ - q0_) * t / duration_;
}
double ease_in_out_sine(double t) {
  return -(std::cos(3.14159265358979323846 * std::clamp(t, 0.0, 1.0)) - 1.0) / 2.0;
}
EaseProfile::EaseProfile(double q0, double qf, double duration)
    : q0_(q0), qf_(qf), duration_(duration) {
  if (duration <= 0.0) {
    throw std::invalid_argument("duration must be positive");
  }
}
double EaseProfile::position(double t) const {
  return q0_ + (qf_ - q0_) * ease_in_out_sine(std::clamp(t, 0.0, duration_) / duration_);
}
double clamp_relative(double goal, double present, double max_diff) {
  return present + std::clamp(goal - present, -max_diff, max_diff);
}

ArmController::ArmController(ServoIO& io, std::vector<std::uint8_t> ids, unsigned fps,
                             double max_step)
    : io_(io), ids_(std::move(ids)), fps_(fps), max_step_(max_step) {
  if (ids_.empty() || fps_ == 0) {
    throw std::invalid_argument("ids and fps must be valid");
  }
}
MotionResult ArmController::move_to(const std::map<std::uint8_t, double>& requested,
                                    const std::string& kind, double v_max, double a_max,
                                    double fixed_duration, bool real_time) {
  MotionResult result;
  std::map<std::uint8_t, double> target;
  std::map<std::uint8_t, TrapezoidalProfile> traps;
  for (const auto id : ids_) {
    result.start[id] = io_.present_position(id);
    const auto [lo, hi] = io_.position_limits(id);
    const auto found = requested.find(id);
    target[id] =
        std::clamp(found == requested.end() ? static_cast<double>(result.start[id]) : found->second,
                   static_cast<double>(lo), static_cast<double>(hi));
    traps.emplace(id, TrapezoidalProfile(result.start[id], target[id], v_max, a_max));
    if (kind == "trapezoid") {
      result.duration = std::max(result.duration, traps.at(id).duration());
    }
  }
  if (kind != "trapezoid") {
    if (kind != "linear" && kind != "ease") {
      throw std::invalid_argument("unknown motion profile");
    }
    if (fixed_duration <= 0.0) {
      for (const auto id : ids_) {
        fixed_duration = std::max(fixed_duration, std::abs(target[id] - result.start[id]) / v_max);
      }
    }
    if (fixed_duration <= 0.0) {
      fixed_duration = 1.0 / fps_;
    }
    result.duration = fixed_duration;
  }
  for (const auto id : ids_) {
    io_.torque(id, true);
  }
  const double dt = 1.0 / fps_;
  std::map<std::uint8_t, int> current = result.start;
  const auto wall_start = std::chrono::steady_clock::now();
  for (double t = 0.0; t < result.duration + dt; t += dt) {
    for (const auto id : ids_) {
      double goal = 0.0;
      if (kind == "trapezoid") {
        goal = traps.at(id).position(t);
      } else if (kind == "linear") {
        goal = LinearProfile(result.start[id], target[id], result.duration).position(t);
      } else {
        goal = EaseProfile(result.start[id], target[id], result.duration).position(t);
      }
      if (max_step_ > 0.0) {
        goal = clamp_relative(goal, current[id], max_step_);
      }
      const int command = static_cast<int>(std::lround(goal));
      io_.goal_position(id, command);
      current[id] = io_.present_position(id);
      result.samples.push_back({t, id, command, current[id]});
    }
    if (real_time) {
      std::this_thread::sleep_until(wall_start +
                                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(t + dt)));
    }
  }
  result.end = current;
  return result;
}
void ArmController::emergency_stop() {
  for (const auto id : ids_) {
    io_.torque(id, false);
  }
}

}  // namespace diyrobot
