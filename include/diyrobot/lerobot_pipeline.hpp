#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace diyrobot {
std::string require_repo_id(const std::string& repo_id);
std::string camera_config(int index, int width, int height, int fps,
                          const std::string& name = "front");
std::string shell_quote(const std::string& value);
std::string render_command(const std::vector<std::string>& arguments);

struct RecordSpec {
  std::string dataset_repo_id, task, follower_port{"COM24"}, leader_port{"COM22"};
  int camera_index{1}, width{1280}, height{720}, fps{30}, episodes{30}, episode_time_s{30},
      reset_time_s{10};
  void validate() const;
  std::vector<std::string> command() const;
};
struct TrainSpec {
  std::string dataset_repo_id, policy{"act"}, output_dir{"outputs/train/act_so101"}, device{"cuda"};
  bool wandb{true};
  std::string policy_repo_id;
  std::vector<std::string> command() const;
};
std::vector<std::string> rollout_command(const std::string& policy_path, const std::string& task,
                                         const std::string& follower_port, int camera_index,
                                         int width, int height, int fps, int duration_s);

struct DatasetReport {
  int episodes{}, frames{}, fps{};
  double duration_s{}, frames_per_episode{};
  std::vector<std::string> camera_features;
  bool has_state{}, has_action{};
};
DatasetReport inspect_local_dataset(const std::filesystem::path& root);
std::vector<std::string> validate_dataset_report(const DatasetReport& report,
                                                 int minimum_episodes = 30);
}  // namespace diyrobot
