#include "diyrobot/lerobot_pipeline.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace diyrobot {
std::string require_repo_id(const std::string& id) {
  static const std::regex valid(R"(^[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*$)");
  if (!std::regex_match(id, valid)) {
    throw std::invalid_argument("repo id must be namespace/name");
  }
  return id;
}
std::string camera_config(int index, int width, int height, int fps, const std::string& name) {
  if (index < 0 || width < 1 || height < 1 || fps < 1 || name.empty()) {
    throw std::invalid_argument("invalid camera configuration");
  }
  std::ostringstream s;
  s << "{" << name << ": {type: opencv, index_or_path: " << index << ", width: " << width
    << ", height: " << height << ", fps: " << fps << "}}";
  return s.str();
}
std::string shell_quote(const std::string& v) {
  if (v.find_first_of(" \t\"'{}") == std::string::npos) {
    return v;
  }
  std::string out = "\"";
  for (char c : v) {
    if (c == '\"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out + '\"';
}
std::string render_command(const std::vector<std::string>& args) {
  std::ostringstream s;
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (i) {
      s << ' ';
    }
    s << shell_quote(args[i]);
  }
  return s.str();
}
void RecordSpec::validate() const {
  require_repo_id(dataset_repo_id);
  if (task.find_first_not_of(" \t") == std::string::npos || follower_port.empty() ||
      leader_port.empty()) {
    throw std::invalid_argument("task and ports are required");
  }
  if (episodes < 1 || episode_time_s < 1 || reset_time_s < 0) {
    throw std::invalid_argument("invalid episode settings");
  }
  camera_config(camera_index, width, height, fps);
}
std::vector<std::string> RecordSpec::command() const {
  validate();
  return {"lerobot-record",
          "--robot.type=so101_follower",
          "--robot.port=" + follower_port,
          "--robot.id=so101_follower",
          "--robot.cameras=" + camera_config(camera_index, width, height, fps),
          "--teleop.type=so101_leader",
          "--teleop.port=" + leader_port,
          "--teleop.id=so101_leader",
          "--dataset.repo_id=" + dataset_repo_id,
          "--dataset.single_task=" + task,
          "--dataset.fps=" + std::to_string(fps),
          "--dataset.num_episodes=" + std::to_string(episodes),
          "--dataset.episode_time_s=" + std::to_string(episode_time_s),
          "--dataset.reset_time_s=" + std::to_string(reset_time_s),
          "--dataset.streaming_encoding=true",
          "--dataset.encoder_threads=2",
          "--display_data=true"};
}
std::vector<std::string> TrainSpec::command() const {
  require_repo_id(dataset_repo_id);
  if (policy != "act" && policy != "diffusion") {
    throw std::invalid_argument("policy must be act or diffusion");
  }
  if (!policy_repo_id.empty()) {
    require_repo_id(policy_repo_id);
  }
  std::filesystem::path out(output_dir);
  std::vector<std::string> result{"lerobot-train",
                                  "--dataset.repo_id=" + dataset_repo_id,
                                  "--policy.type=" + policy,
                                  "--output_dir=" + output_dir,
                                  "--job_name=" + out.filename().string(),
                                  "--policy.device=" + device,
                                  "--wandb.enable=" + std::string(wandb ? "true" : "false")};
  if (!policy_repo_id.empty()) {
    result.push_back("--policy.repo_id=" + policy_repo_id);
  }
  return result;
}
std::vector<std::string> rollout_command(const std::string& p, const std::string& t,
                                         const std::string& port, int cam, int w, int h, int fps,
                                         int seconds) {
  if (p.empty() || t.empty() || port.empty() || seconds < 1) {
    throw std::invalid_argument("rollout fields are required");
  }
  return {"lerobot-rollout",      "--strategy.type=base",
          "--policy.path=" + p,   "--robot.type=so101_follower",
          "--robot.port=" + port, "--robot.cameras=" + camera_config(cam, w, h, fps),
          "--task=" + t,          "--duration=" + std::to_string(seconds)};
}
namespace {
int integer_field(const std::string& s, const std::string& key) {
  std::smatch m;
  std::regex r("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
  return std::regex_search(s, m, r) ? std::stoi(m[1].str()) : 0;
}
}  // namespace
DatasetReport inspect_local_dataset(const std::filesystem::path& root) {
  const auto path = root / "meta" / "info.json";
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("LeRobot metadata not found: " + path.string());
  }
  const std::string text((std::istreambuf_iterator<char>(in)), {});
  DatasetReport report;
  report.episodes = integer_field(text, "total_episodes");
  report.frames = integer_field(text, "total_frames");
  report.fps = integer_field(text, "fps");
  report.duration_s = report.fps ? static_cast<double>(report.frames) / report.fps : 0.0;
  report.frames_per_episode =
      report.episodes ? static_cast<double>(report.frames) / report.episodes : 0.0;
  report.has_state = text.find("\"observation.state\"") != std::string::npos;
  report.has_action = text.find("\"action\"") != std::string::npos;
  const std::regex cameras(R"regex("([^"]+)"\s*:\s*\{[^{}]*"dtype"\s*:\s*"(video|image)")regex");
  for (std::sregex_iterator it(text.begin(), text.end(), cameras), end; it != end; ++it) {
    report.camera_features.push_back((*it)[1].str());
  }
  return report;
}
std::vector<std::string> validate_dataset_report(const DatasetReport& r, int minimum) {
  std::vector<std::string> e;
  if (r.episodes < minimum) {
    e.push_back("not enough episodes");
  }
  if (r.fps <= 0 || r.frames_per_episode < r.fps * 3) {
    e.push_back("episodes too short or fps invalid");
  }
  if (r.camera_features.empty()) {
    e.push_back("camera feature missing");
  }
  if (!r.has_state || !r.has_action) {
    e.push_back("state or action missing");
  }
  return e;
}
}  // namespace diyrobot
