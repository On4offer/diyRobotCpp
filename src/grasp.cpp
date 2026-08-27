#include "diyrobot/grasp.hpp"

#include <cmath>

namespace diyrobot {
GraspController::GraspController(const SO101Kinematics& kin, const TableCalibration& cal,
                                 const ColorTargetDetector& detector, GraspHardware& hardware,
                                 GraspConfig config)
    : kinematics_(kin),
      calibration_(cal),
      detector_(detector),
      hardware_(hardware),
      config_(config) {}
void GraspController::enter(GraspState state) {
  trace_.push_back(state);
}
bool GraspController::jaw_holds_object(int open_position) const {
  const int present = hardware_.gripper_position();
  const int moved = std::abs(present - open_position);
  if (moved < config_.minimum_motion) {
    return false;
  }
  if (hardware_.gripper_overloaded()) {
    return true;
  }
  return std::abs(present - config_.gripper_close_code) > config_.close_tolerance;
}

bool GraspController::recover_safely() {
  enter(GraspState::recover);
  try {
    hardware_.command_gripper(config_.gripper_open_code);
    hardware_.safe_home();
    return true;
  } catch (const std::exception&) {
    try {
      hardware_.emergency_stop();
    } catch (const std::exception&) {
      // Both recovery paths failed. The caller records this separately so the
      // failure remains visible to an operator instead of being swallowed.
    }
    return false;
  }
}

bool GraspController::grasp_once(const Image& frame) {
  ++stats_.trials;
  trace_.clear();
  enter(GraspState::idle);
  enter(GraspState::detect);
  const auto target = detector_.detect_one(frame);
  if (!target) {
    ++stats_.failed;
    ++stats_.failure_reasons["detect_fail"];
    enter(GraspState::failed);
    return false;
  }
  const auto base = calibration_.pixel_to_base(target->x, target->y);
  if (std::hypot(base[0], base[1]) > config_.max_reach) {
    ++stats_.failed;
    ++stats_.failure_reasons["out_of_reach"];
    enter(GraspState::failed);
    return false;
  }
  for (unsigned attempt = 0; attempt <= config_.max_retries; ++attempt) {
    try {
      hardware_.command_gripper(config_.gripper_open_code);
      enter(GraspState::approach);
      hardware_.move_joints(kinematics_.ik_vertical(base[0], base[1], config_.z_pre));
      enter(GraspState::descend);
      hardware_.move_joints(kinematics_.ik_vertical(base[0], base[1], config_.z_grasp));
      const int open_position = hardware_.gripper_position();
      enter(GraspState::close);
      hardware_.command_gripper(config_.gripper_close_code);
      enter(GraspState::lift);
      hardware_.move_joints(kinematics_.ik_vertical(base[0], base[1], config_.z_lift));
      enter(GraspState::verify);
      if (jaw_holds_object(open_position)) {
        enter(GraspState::place);
        hardware_.move_joints(
            kinematics_.ik_vertical(config_.place_xy[0], config_.place_xy[1], config_.z_pre));
        hardware_.command_gripper(config_.gripper_open_code);
        hardware_.safe_home();
        ++stats_.success;
        enter(GraspState::done);
        return true;
      }
      if (!recover_safely()) {
        ++stats_.failure_reasons["recovery_failed"];
        break;
      }
    } catch (const KinematicsError&) {
      ++stats_.failure_reasons["kinematics_error"];
      if (!recover_safely()) {
        ++stats_.failure_reasons["recovery_failed"];
      }
      break;
    } catch (const std::exception&) {
      ++stats_.failure_reasons["hardware_error"];
      if (!recover_safely()) {
        ++stats_.failure_reasons["recovery_failed"];
      }
      break;
    }
  }
  ++stats_.failed;
  ++stats_.failure_reasons["grasp_fail"];
  enter(GraspState::failed);
  return false;
}
const char* to_string(GraspState s) {
  switch (s) {
    case GraspState::idle:
      return "idle";
    case GraspState::detect:
      return "detect";
    case GraspState::approach:
      return "approach";
    case GraspState::descend:
      return "descend";
    case GraspState::close:
      return "close";
    case GraspState::lift:
      return "lift";
    case GraspState::verify:
      return "verify";
    case GraspState::place:
      return "place";
    case GraspState::recover:
      return "recover";
    case GraspState::done:
      return "done";
    case GraspState::failed:
      return "failed";
  }
  return "unknown";
}
}  // namespace diyrobot
