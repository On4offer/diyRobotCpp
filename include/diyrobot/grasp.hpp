#pragma once

#include <map>
#include <string>
#include <vector>

#include "diyrobot/calibration.hpp"
#include "diyrobot/kinematics.hpp"
#include "diyrobot/vision.hpp"

namespace diyrobot {

enum class GraspState {
  idle,
  detect,
  approach,
  descend,
  close,
  lift,
  verify,
  place,
  recover,
  done,
  failed
};
struct GraspConfig {
  double z_pre{0.14}, z_grasp{0.08}, z_lift{0.16}, max_reach{0.30};
  Point2 place_xy{0.06, 0.10};
  int gripper_open_code{2426}, gripper_close_code{1560};
  int close_tolerance{80}, minimum_motion{40};
  unsigned max_retries{2};
};
struct GraspStats {
  unsigned trials{}, success{}, failed{};
  std::map<std::string, unsigned> failure_reasons;
};

class GraspHardware {
 public:
  virtual ~GraspHardware() = default;
  virtual void move_joints(const JointMap& joints) = 0;
  virtual void command_gripper(int code) = 0;
  virtual int gripper_position() = 0;
  virtual bool gripper_overloaded() = 0;
  virtual void safe_home() = 0;
  virtual void emergency_stop() = 0;
};

class GraspController {
 public:
  GraspController(const SO101Kinematics& kinematics, const TableCalibration& calibration,
                  const ColorTargetDetector& detector, GraspHardware& hardware,
                  GraspConfig config = {});
  bool grasp_once(const Image& frame);
  const GraspStats& stats() const {
    return stats_;
  }
  const std::vector<GraspState>& trace() const {
    return trace_;
  }

 private:
  void enter(GraspState state);
  bool jaw_holds_object(int open_position) const;
  bool recover_safely();
  const SO101Kinematics& kinematics_;
  const TableCalibration& calibration_;
  const ColorTargetDetector& detector_;
  GraspHardware& hardware_;
  GraspConfig config_;
  GraspStats stats_;
  std::vector<GraspState> trace_;
};

const char* to_string(GraspState state);
}  // namespace diyrobot
