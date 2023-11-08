/* ROS2 Humble
-Create a Node that gets the latest transform between rick/base_link and morty/base_link
-validate the transform by printing it to the screen with RCLCPP_INFO
-Calculate the distance between the two robots and the angle between them
-Use a proportional controller to make rick chase morty
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/msg/twist.hpp>

class RobotChase : public rclcpp::Node {
public:
  RobotChase(const std::string &origin_frame, const std::string &destiny_frame)
      : Node("robot_chase"), TIMER_PERIOD_(0.05), MAX_LINEAR_SPEED_(0.5),
        MAX_ANGULAR_SPEED_(0.5), kp_distance_(0.5), kp_yaw_(0.5) {
    // initialize the tf2 listener
    auto clock = this->get_clock();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    origin_frame_ = origin_frame;
    destiny_frame_ = destiny_frame;
    // initialize the timer
    timer_period_ms = static_cast<int>(1.0 / TIMER_PERIOD_);
    timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_ms),
                               std::bind(&RobotChase::timer_callback, this));
    // publisher to cmd_vel of hunter
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
  }

private:
  // tf2 listener variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string origin_frame_, destiny_frame_;
  // timer variables
  rclcpp::TimerBase::SharedPtr timer_;
  const float TIMER_PERIOD_;
  int timer_period_ms;
  // publisher variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist pub_msg_;
  const float MAX_LINEAR_SPEED_;
  const float MAX_ANGULAR_SPEED_;
  // control variables
  float error_distance_, error_yaw_, error_direction_;
  const float kp_distance_;
  const float kp_yaw_;

  void timer_callback() {
    // 1st get the transform
    geometry_msgs::msg::Vector3 translation;
    geometry_msgs::msg::Quaternion rotation;
    std::tie(translation, rotation) = get_transform();

    // 2nd calculate the distance and angle between the two robots
    error_distance_ = calculate_tf_distance(translation, false);
    error_yaw_ = calculate_tf_yaw(rotation, true, false);
    error_direction_ = calculate_tf_direction(translation, false, true);

    //3rd perform the movement
    /*orientation control: 
    -control variable : direction defined by the vector between the two robots
    -desired variable : values close to 0
    */
    pub_msg_.angular.z = kp_yaw_ * error_direction_;
    //distance control
    //pub_msg_.linear.x = kp_distance_ * error_distance_;
    //publish the message
    publisher_->publish(pub_msg_);

  }

  double calculate_tf_yaw(geometry_msgs::msg::Quaternion rotation,
                          bool in_degree = true, bool debug = false) {
    tf2::Quaternion tf2_rotation;
    tf2_rotation.setX(rotation.x);
    tf2_rotation.setY(rotation.y);
    tf2_rotation.setZ(rotation.z);
    tf2_rotation.setW(rotation.w);
    tf2::Matrix3x3 m(tf2_rotation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (in_degree) {
      yaw = yaw * 180 / M_PI;
    }
    if (debug) {
      RCLCPP_INFO(this->get_logger(), "yaw angle between frames: %.2f", yaw);
    }
    return yaw;
  }

  double calculate_tf_distance(geometry_msgs::msg::Vector3 translation,
                               bool debug = false) {
    float distance = sqrt(pow(translation.x, 2) + pow(translation.y, 2));
    if (debug) {
      RCLCPP_INFO(this->get_logger(), "distance between frames: %.2f",
                  distance);
    }
    return distance;
  }

  double calculate_tf_direction(geometry_msgs::msg::Vector3 translation,
                                bool in_degree = true, bool debug = false) {
    double direction = atan2(translation.y, translation.x);
    if (in_degree) {
      direction = direction * 180 / M_PI;
    }
    if (debug) {
      RCLCPP_INFO(this->get_logger(), "direction between frames: %.2f",
                  direction);
    }
    return direction;
  }

  std::tuple<geometry_msgs::msg::Vector3, geometry_msgs::msg::Quaternion>
  get_transform() {
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::Vector3 translation;
    geometry_msgs::msg::Quaternion rotation;

    try {
      transformStamped = tf_buffer_->lookupTransform(
          origin_frame_, destiny_frame_, tf2::TimePointZero);
      translation = transformStamped.transform.translation;
      rotation = transformStamped.transform.rotation;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  origin_frame_.c_str(), destiny_frame_.c_str(), ex.what());
    }

    return std::make_tuple(translation, rotation);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<RobotChase>("rick/base_link", "morty/base_link"));
  rclcpp::shutdown();
  return 0;
}