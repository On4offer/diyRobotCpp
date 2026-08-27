#include <geometry_msgs/msg/point.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

#include "diyrobot/calibration.hpp"
#include "diyrobot/kinematics.hpp"

class GraspNode final : public rclcpp::Node {
 public:
  GraspNode() : Node("diyrobot_grasp_node"), kinematics_({}, {}), calibration_() {
    const bool dry_run = declare_parameter("dry_run", true);
    declare_parameter("z_pre", 0.14);
    declare_parameter("z_grasp", 0.08);
    const auto homography = declare_parameter<std::vector<double>>(
        "homography", {0.0002, 0.0, 0.08, 0.0, -0.0005, 0.12, 0.0, 0.0, 1.0});
    if (!dry_run) {
      throw std::invalid_argument(
          "dry_run=false is unsupported: a real ROS 2 hardware backend is not implemented");
    }
    if (homography.size() != 9) {
      throw std::invalid_argument("homography must contain exactly 9 row-major values");
    }
    calibration_.set_homography({{{{homography[0], homography[1], homography[2]}},
                                  {{homography[3], homography[4], homography[5]}},
                                  {{homography[6], homography[7], homography[8]}}}});
    target_sub_ = create_subscription<geometry_msgs::msg::Point>(
        "target_pixel", 10, [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
          last_target_ = *msg;
          publish_state("target_received");
        });
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("grasp_state", 10);
    trigger_ = create_service<std_srvs::srv::Trigger>(
        "start_grasp", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                              const std_srvs::srv::Trigger::Response::SharedPtr response) {
          run_grasp(*response);
        });
    publish_state("ready");
    RCLCPP_INFO(get_logger(), "diyRobotCpp grasp node ready (dry_run=true)");
  }

 private:
  void publish_state(const std::string& value) {
    std_msgs::msg::String msg;
    msg.data = value;
    state_pub_->publish(msg);
  }
  void publish_pose(const diyrobot::JointMap& pose) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (const auto& [name, value] : pose) {
      msg.name.push_back(name + "_joint");
      msg.position.push_back(value * 3.14159265358979323846 / 180.0);
    }
    joint_pub_->publish(msg);
  }
  void run_grasp(std_srvs::srv::Trigger::Response& response) {
    if (!last_target_) {
      response.success = false;
      response.message = "no target_pixel received";
      return;
    }
    try {
      const auto base = calibration_.pixel_to_base(last_target_->x, last_target_->y);
      publish_state("approach");
      publish_pose(kinematics_.ik_vertical(base[0], base[1], get_parameter("z_pre").as_double()));
      publish_state("descend");
      publish_pose(kinematics_.ik_vertical(base[0], base[1], get_parameter("z_grasp").as_double()));
      publish_state("dry_run_complete");
      response.success = true;
      response.message = "offline trajectory published; no hardware command was sent";
    } catch (const std::exception& e) {
      publish_state("failed");
      response.success = false;
      response.message = e.what();
    }
  }
  diyrobot::SO101Kinematics kinematics_;
  diyrobot::TableCalibration calibration_;
  std::optional<geometry_msgs::msg::Point> last_target_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspNode>());
  rclcpp::shutdown();
}
