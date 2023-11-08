/* ROS2 Humble
-Create a Node that gets the latest transform between rick/base_link and morty/base_link
-validate the transform by printing it to the screen with RCLCPP_INFO
-Calculate the distance between the two robots and the angle between them
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

class RobotChase : public rclcpp::Node
{
public:
    RobotChase() : Node("robot_chase"), TIMER_PERIOD_(0.05)
    {
        auto clock = this->get_clock();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        timer_period_ms = static_cast<int>(1.0 / TIMER_PERIOD_);
        timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_ms), std::bind(&RobotChase::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    const float TIMER_PERIOD_;
    int timer_period_ms;
    float error_distance_, error_yaw_, error_direction_;
    
    void timer_callback()
    {
        //1st get the transform
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::Vector3 translation;
        geometry_msgs::msg::Quaternion rotation;

        try
        {
            transformStamped = tf_buffer_->lookupTransform("rick/base_link", "morty/base_link", tf2::TimePointZero);
            translation = transformStamped.transform.translation;
            rotation = transformStamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", "morty/base_link", "rick/base_link", ex.what());
            return;
        }

        //2nd calculate the distance and angle between the two robots
        error_distance_ = calculate_tf_distance(translation,true);
        error_yaw_ = calculate_tf_yaw(rotation,true,true);
        error_direction_ = calculate_tf_direction(translation,true,true);

    }

    double calculate_tf_yaw(geometry_msgs::msg::Quaternion rotation, bool in_degree = true, bool debug=false)
    {
        tf2::Quaternion tf2_rotation;
        tf2_rotation.setX(rotation.x);
        tf2_rotation.setY(rotation.y);
        tf2_rotation.setZ(rotation.z);
        tf2_rotation.setW(rotation.w);
        tf2::Matrix3x3 m(tf2_rotation);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (in_degree)
        {
            yaw=yaw * 180 / M_PI;
        }
        if (debug)
        {
            RCLCPP_INFO(this->get_logger(), "yaw angle between frames: %.2f", yaw);
        }
        return yaw;
    }

    double calculate_tf_distance(geometry_msgs::msg::Vector3 translation, bool debug=false)
    {
        float distance = sqrt(pow(translation.x, 2) + pow(translation.y, 2));
        if (debug)
        {
            RCLCPP_INFO(this->get_logger(), "distance between frames: %.2f", distance);
        }
        return distance;
    }

    double calculate_tf_direction(geometry_msgs::msg::Vector3 translation, bool in_degree = true, bool debug=false)
    {
        double direction = atan2(translation.y, translation.x);
        if (in_degree)
        {
            direction=direction * 180 / M_PI;
        }
        if (debug)
        {
            RCLCPP_INFO(this->get_logger(), "direction between frames: %.2f", direction);
        }
        return direction;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotChase>());
    rclcpp::shutdown();
    return 0;
}