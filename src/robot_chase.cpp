#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&RobotChase::controlRick, this));

    kp_distance_ = 0.4;
    ki_distance_ = 0;
    kd_distance_ = 0;

    kp_angle_ = 0.4;
    ki_angle_ = 0;
    kd_angle_ = 0;

    prev_error_distance_ = 0.0;
    integral_distance_ = 0.0;

    prev_error_angle_ = 0.0;
    integral_angle_ = 0.0;
    max_linear_speed = 0.5;
    max_angular_speed = 0.5;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_linear_speed;
  double max_angular_speed;
  double kp_distance_;
  double ki_distance_;
  double kd_distance_;
  double prev_error_distance_;
  double integral_distance_;
  double kp_angle_;
  double ki_angle_;
  double kd_angle_;
  double prev_error_angle_;
  double integral_angle_;

  void controlRick() {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s",
                   ex.what());
      return;
    }

    double error_distance = calculateDistanceError(transform);
    double error_angle = calculateAngleError(transform);

    RCLCPP_INFO(get_logger(), "Distance: %f ; Angle: %f", error_distance,
                error_angle);

    double pid_distance =
        (kp_distance_ * error_distance) + (ki_distance_ * integral_distance_) +
        kd_distance_ * (error_distance - prev_error_distance_);

    double pid_angle = (kp_angle_ * error_angle) +
                       (ki_angle_ * integral_angle_) +
                       kd_angle_ * (error_angle - prev_error_angle_);

    pid_distance =std::min(max_linear_speed, std::max(-max_linear_speed,
     pid_distance));
     pid_angle =  std::min(max_angular_speed, std::max(-max_angular_speed,
     pid_angle));

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = pid_angle;
    cmd_vel.linear.x = pid_distance;
    publisher_->publish(cmd_vel);

    prev_error_distance_ = error_distance;
    integral_distance_ += error_distance;
    prev_error_angle_ = error_angle;
    integral_angle_ += error_angle;
  }

  double calculateDistanceError(
      const geometry_msgs::msg::TransformStamped &transform) {
    double error_distance = sqrt(pow(transform.transform.translation.x, 2) +
                                 pow(transform.transform.translation.y, 2));
    return error_distance;
  }

  double
  calculateAngleError(const geometry_msgs::msg::TransformStamped &transform) {
    tf2::Quaternion quat;
    tf2::fromMsg(transform.transform.rotation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double error_angle = atan2(transform.transform.translation.y,
                               transform.transform.translation.x);
    return error_angle;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
