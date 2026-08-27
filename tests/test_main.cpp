#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "diyrobot/calibration.hpp"
#include "diyrobot/grasp.hpp"
#include "diyrobot/kinematics.hpp"
#include "diyrobot/lerobot_pipeline.hpp"
#include "diyrobot/motion.hpp"
#include "diyrobot/protocol.hpp"
#include "diyrobot/vision.hpp"

namespace {
using namespace diyrobot;
void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}
void near(double actual, double expected, double tolerance, const std::string& message) {
  require(std::abs(actual - expected) <= tolerance, message + " actual=" + std::to_string(actual));
}
CalibrationMap calibration() {
  return {{"shoulder_pan", {2095, 814, 3376}}, {"shoulder_lift", {2036.5, 841, 3232}},
          {"elbow_flex", {1969.5, 866, 3073}}, {"wrist_flex", {2029, 905, 3153}},
          {"wrist_roll", {2047.5, 0, 4095}},   {"gripper", {1993, 1560, 2426}}};
}
class FakeHardware final : public GraspHardware {
 public:
  explicit FakeHardware(int stall = 0, bool frozen = false) : stall_(stall), frozen_(frozen) {}
  void move_joints(const JointMap& j) override {
    moves.push_back(j);
  }
  void command_gripper(int code) override {
    if (code > 2000) {
      position_ = code;
    } else if (!frozen_) {
      position_ = code + stall_;
    }
  }
  int gripper_position() override {
    return position_;
  }
  bool gripper_overloaded() override {
    return stall_ > 0;
  }
  void safe_home() override {
    if (fail_home) {
      throw std::runtime_error("home failed");
    }
    ++homes;
  }
  void emergency_stop() override {
    ++emergency_stops;
  }
  int stall_{}, position_{2426};
  bool frozen_{};
  bool fail_move{};
  bool fail_home{};
  int homes{};
  int emergency_stops{};
  std::vector<JointMap> moves;
};
class FakeServoIO final : public ServoIO {
 public:
  int present_position(std::uint8_t id) override {
    return positions[id];
  }
  std::pair<int, int> position_limits(std::uint8_t id) override {
    return limits[id];
  }
  void goal_position(std::uint8_t id, int value) override {
    positions[id] = value;
  }
  void torque(std::uint8_t id, bool enabled) override {
    torque_enabled[id] = enabled;
  }
  std::map<std::uint8_t, int> positions{{1, 50}};
  std::map<std::uint8_t, std::pair<int, int>> limits{{1, {0, 100}}};
  std::map<std::uint8_t, bool> torque_enabled;
};
struct Test {
  std::string name;
  std::function<void()> run;
};
}  // namespace
int main() {
  using namespace diyrobot;
  std::vector<Test> tests;
  auto add = [&](std::string name, std::function<void()> fn) {
    tests.push_back({std::move(name), std::move(fn)});
  };

  add("protocol ping frame", [] {
    require(build_instruction_packet(1, kInstPing) ==
                std::vector<std::uint8_t>({0xff, 0xff, 1, 2, 1, 0xfb}),
            "ping bytes");
  });
  add("protocol read frame", [] {
    require(build_instruction_packet(1, kInstRead, {0x2a, 2}) ==
                std::vector<std::uint8_t>({0xff, 0xff, 1, 4, 2, 0x2a, 2, 0xcc}),
            "read bytes");
  });
  add("protocol status parse", [] {
    auto s = parse_status_packet({0xff, 0xff, 1, 4, 0, 0x80, 8, 0x72});
    require(s.servo_id == 1 && s.params == std::vector<std::uint8_t>({0x80, 8}), "status");
  });
  add("protocol rejects checksum", [] {
    try {
      parse_status_packet({0xff, 0xff, 1, 4, 0, 0x80, 8, 0});
      throw std::runtime_error("accepted");
    } catch (const ProtocolError&) {
    };
  });
  add("protocol stream recovery", [] {
    auto p = parse_status_stream({0xaa, 0xff, 0xff, 1, 2, 0, 0xfc, 0xff, 0xff, 2, 2, 0, 0xfb});
    require(p.size() == 2 && p[1].servo_id == 2, "stream");
  });
  add("sign magnitude", [] {
    for (int v : {-4095, -1, 0, 1, 4095}) {
      require(decode_sign_magnitude(encode_sign_magnitude(v)) == v, "roundtrip");
    }
  });
  add("u16 endian", [] {
    auto p = split_u16(0x0880);
    require(p.first == 0x80 && p.second == 8 && join_u16(p.first, p.second) == 0x0880, "endian");
  });
  add("angle counts", [] {
    require(angle_to_counts(30) == 341, "angle");
    near(counts_to_angle(341), 29.9707, 0.01, "counts");
  });

  add("trapezoid endpoints", [] {
    TrapezoidalProfile p(0, 1000, 200, 400);
    near(p.position(0), 0, 0, "start");
    near(p.position(p.duration()), 1000, 0, "end");
    near(p.duration(), 5.5, 1e-9, "duration");
  });
  add("trapezoid bounds", [] {
    TrapezoidalProfile p(0, 1000, 200, 400);
    for (double t = 0; t <= p.duration(); t += .001) {
      require(std::abs(p.velocity(t)) <= 200.0001, "speed");
    }
  });
  add("trapezoid negative", [] {
    TrapezoidalProfile p(1000, 200, 200, 400);
    near(p.velocity(p.acceleration_time()), -200, 1e-9, "negative");
  });
  add("trapezoid zero", [] {
    TrapezoidalProfile p(500, 500, 200, 400);
    near(p.position(1), 500, 0, "zero");
  });
  add("linear profile", [] {
    LinearProfile p(0, 100, 2);
    near(p.position(1), 50, 1e-12, "linear");
  });
  add("ease profile", [] {
    EaseProfile p(0, 100, 2);
    near(p.position(1), 50, 1e-9, "ease");
    near(ease_in_out_sine(0), 0, 0, "ease start");
  });
  add("relative clamp", [] {
    near(clamp_relative(1000, 0, 100), 100, 0, "positive");
    near(clamp_relative(-1000, 0, 100), -100, 0, "negative");
  });
  add("arm controller limits and estop", [] {
    FakeServoIO io;
    ArmController arm(io, {1}, 10, 5);
    auto result = arm.move_to({{1, 200}}, "linear", 100, 200, 1, false);
    require(result.end.at(1) == 100 && io.torque_enabled.at(1), "clipped move");
    arm.emergency_stop();
    require(!io.torque_enabled.at(1), "estop");
  });

  add("kinematics code roundtrip", [] {
    SO101Kinematics k({}, calibration());
    for (const auto& j : {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"}) {
      for (double d : {-30.0, 0.0, 30.0}) {
        near(k.code_to_deg(j, k.deg_to_code(j, d)), d, .1, "joint");
      }
    }
  });
  add("kinematics ik fk", [] {
    SO101Kinematics k({}, calibration());
    for (auto p :
         std::vector<std::array<double, 3>>{{.15, .05, .08}, {.20, -.05, .02}, {.10, 0, .005}}) {
      auto xyz = k.fk(k.ik_vertical(p[0], p[1], p[2]));
      near(std::hypot(std::hypot(xyz[0] - p[0], xyz[1] - p[1]), xyz[2] - p[2]), 0, 1e-3, "ik/fk");
    }
  });
  add("kinematics unreachable", [] {
    SO101Kinematics k({}, calibration());
    try {
      k.ik_vertical(1, 0, .05);
      throw std::runtime_error("accepted");
    } catch (const KinematicsError&) {
    };
  });
  add("kinematics zero fk", [] {
    SO101Kinematics k({}, calibration());
    auto p =
        k.fk({{"shoulder_pan", 0}, {"shoulder_lift", 0}, {"elbow_flex", 0}, {"wrist_flex", 0}});
    near(p[0], .33, .02, "x");
    near(p[2], .23, .02, "z");
  });
  add("kinematics safe path", [] {
    SO101Kinematics k({}, calibration());
    auto a = k.ik_vertical(.10, 0, .14), b = k.ik_vertical(.16, 0, .14);
    k.check_path_safe(a, b, .075);
  });
  add("kinematics unsafe path", [] {
    SO101Kinematics k({}, calibration());
    JointMap a{
        {"shoulder_pan", 0}, {"shoulder_lift", -20}, {"elbow_flex", 130}, {"wrist_flex", 90}},
        b = k.ik_vertical(.15, 0, .10);
    try {
      k.check_path_safe(a, b, .075);
      throw std::runtime_error("accepted");
    } catch (const PathUnsafeError&) {
    };
  });

  add("calibration exact solve", [] {
    TableCalibration c;
    for (auto b : grid_xy(.08, .24, 3, -.08, .08, 3)) {
      const double u = 800 * b[0] + 120 * b[1] + 320, v = -50 * b[0] + 900 * b[1] + 240;
      c.add_point({u, v}, b);
    }
    auto e = c.solve();
    require(e.rms_millimeters < .01 && e.rms_pixels < .001, "reprojection");
  });
  add("calibration roundtrip", [] {
    TableCalibration c;
    c.set_homography({{{{.001, 0, -.2}}, {{0, .001, -.1}}, {{0, 0, 1}}}});
    auto p = c.pixel_to_base(350, 200);
    auto q = c.base_to_pixel(p[0], p[1]);
    near(q[0], 350, 1e-9, "u");
    near(q[1], 200, 1e-9, "v");
  });
  add("calibration insufficient", [] {
    TableCalibration c;
    for (int i = 0; i < 3; ++i) {
      c.add_point({double(i), 0}, {double(i), 0});
    }
    try {
      c.solve();
      throw std::runtime_error("accepted");
    } catch (const CalibrationError&) {
    };
  });
  add("calibration grid", [] {
    auto g = grid_xy(.1, .24, 3, -.08, .08, 3);
    require(g.size() == 9, "size");
    near(g.back()[0], .24, 0, "x");
  });

  add("vision detects red", [] {
    Image f(320, 240, {240, 240, 240});
    f.fill_rect(100, 80, 60, 60, {0, 0, 255});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    auto t = d.detect_one(f);
    require(t && std::abs(t->x - 130) <= 1 && std::abs(t->y - 110) <= 1, "red");
  });
  add("vision hsv wrap", [] {
    Image f(100, 100);
    f.fill_rect(20, 20, 40, 40, {0, 0, 255});
    ColorTargetDetector d({170, 10, 80, 255, 60, 255}, 100, 50000);
    require(d.detect_one(f).has_value(), "wrap");
  });
  add("vision missing", [] {
    Image f(100, 100);
    f.fill_rect(20, 20, 40, 40, {0, 255, 0});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    require(!d.detect_one(f), "missing");
  });
  add("vision area filter", [] {
    Image f(100, 100);
    f.fill_rect(20, 20, 3, 3, {0, 0, 255});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    require(!d.detect_one(f), "small");
  });
  add("vision largest", [] {
    Image f(320, 240);
    f.fill_rect(20, 20, 30, 30, {0, 0, 255});
    f.fill_rect(160, 120, 80, 80, {0, 0, 255});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    auto t = d.detect_one(f);
    require(t && std::abs(t->x - 200) <= 1, "largest");
  });

  add("grasp success on stall", [] {
    SO101Kinematics k({}, calibration());
    TableCalibration c;
    c.set_homography({{{{.0002, 0, .08}}, {{0, -.0005, .12}}, {{0, 0, 1}}}});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    FakeHardware h(250);
    GraspController g(k, c, d, h);
    Image f(640, 480);
    f.fill_rect(380, 220, 40, 40, {0, 0, 255});
    require(g.grasp_once(f) && g.stats().success == 1, "success");
  });
  add("grasp empty fails", [] {
    SO101Kinematics k({}, calibration());
    TableCalibration c;
    c.set_homography({{{{.0002, 0, .08}}, {{0, -.0005, .12}}, {{0, 0, 1}}}});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    FakeHardware h;
    GraspController g(k, c, d, h);
    Image f(640, 480);
    f.fill_rect(380, 220, 40, 40, {0, 0, 255});
    require(!g.grasp_once(f) && g.stats().failure_reasons.at("grasp_fail") == 1, "empty");
  });
  add("grasp detect fail", [] {
    SO101Kinematics k({}, calibration());
    TableCalibration c;
    c.set_homography({{{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}}});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    FakeHardware h(250);
    GraspController g(k, c, d, h);
    Image f(100, 100);
    require(!g.grasp_once(f) && g.stats().failure_reasons.at("detect_fail") == 1, "detect fail");
  });
  add("grasp frozen fails", [] {
    SO101Kinematics k({}, calibration());
    TableCalibration c;
    c.set_homography({{{{.0002, 0, .08}}, {{0, -.0005, .12}}, {{0, 0, 1}}}});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    FakeHardware h(0, true);
    GraspController g(k, c, d, h);
    Image f(640, 480);
    f.fill_rect(380, 220, 40, 40, {0, 0, 255});
    require(!g.grasp_once(f), "frozen");
  });
  add("grasp recovery falls back to emergency stop", [] {
    SO101Kinematics k({}, calibration());
    TableCalibration c;
    c.set_homography({{{{.0002, 0, .08}}, {{0, -.0005, .12}}, {{0, 0, 1}}}});
    ColorTargetDetector d({0, 10, 80, 255, 60, 255}, 100, 50000);
    FakeHardware h;
    h.fail_home = true;
    GraspController g(k, c, d, h);
    Image f(640, 480);
    f.fill_rect(380, 220, 40, 40, {0, 0, 255});
    require(!g.grasp_once(f), "recovery failure must fail the grasp");
    require(h.emergency_stops == 1, "emergency stop fallback");
    require(g.stats().failure_reasons.at("recovery_failed") == 1, "visible recovery error");
  });

  add("record command contract", [] {
    RecordSpec s;
    s.dataset_repo_id = "On4offer/so101_pick";
    s.task = "Pick block";
    auto c = s.command();
    require(std::find(c.begin(), c.end(), "--dataset.num_episodes=30") != c.end(), "episodes");
  });
  add("invalid repo rejected", [] {
    RecordSpec s;
    s.dataset_repo_id = "bad repo";
    s.task = "Pick";
    try {
      s.command();
      throw std::runtime_error("accepted");
    } catch (const std::invalid_argument&) {
    };
  });
  add("train policies", [] {
    TrainSpec s;
    s.dataset_repo_id = "On4offer/data";
    require(render_command(s.command()).find("--policy.type=act") != std::string::npos, "act");
    s.policy = "diffusion";
    require(render_command(s.command()).find("diffusion") != std::string::npos, "diffusion");
  });
  add("rollout bounded", [] {
    auto c = rollout_command("out/model", "Pick", "COM24", 1, 640, 480, 30, 20);
    require(std::find(c.begin(), c.end(), "--duration=20") != c.end(), "duration");
  });
  add("dataset quality gate", [] {
    DatasetReport r;
    r.episodes = 2;
    r.fps = 30;
    r.frames_per_episode = 20;
    r.has_action = true;
    require(validate_dataset_report(r).size() == 4, "four errors");
  });
  add("dataset metadata inspection", [] {
    auto root = std::filesystem::temp_directory_path() / "diyrobot_cpp_dataset_test";
    std::filesystem::create_directories(root / "meta");
    std::ofstream(root / "meta" / "info.json")
        << R"({"total_episodes":30,"total_frames":27000,"fps":30,)"
        << R"("features":{"observation.images.front":{"dtype":"video"},)"
        << R"("observation.state":{"dtype":"float32"},)"
        << R"("action":{"dtype":"float32"}}})";
    auto r = inspect_local_dataset(root);
    std::filesystem::remove_all(root);
    require(r.episodes == 30 && r.camera_features.size() == 1 && validate_dataset_report(r).empty(),
            "metadata");
  });

  unsigned passed = 0;
  for (const auto& t : tests) {
    try {
      t.run();
      std::cout << "PASS  " << t.name << '\n';
      ++passed;
    } catch (const std::exception& e) {
      std::cerr << "FAIL  " << t.name << ": " << e.what() << '\n';
    }
  }
  std::cout << "\nSummary: " << passed << " PASS / " << (tests.size() - passed) << " FAIL / "
            << tests.size() << " checks\n";
  return passed == tests.size() ? 0 : 1;
}
