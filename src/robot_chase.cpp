/* ROS2 Humble
-Create a Node that gets the latest transform between rick/base_link and morty/base_link
-validate the transform by printing it to the screen with RCLCPP_INFO
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class RobotChase : public rclcpp::Node
{
public:
    RobotChase() : Node("robot_chase")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotChase::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    void timer_callback()
    {
        try
        {
            auto transformStamped = tf_buffer_->lookupTransform("morty/base_link", "rick/base_link", tf2::TimePointZero);
            auto translation = transformStamped.transform.translation;
            auto rotation = transformStamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", origin_frame, dest_frame, ex.what());
            //return;
        }
        //print [x,y,theta°]
        auto theta = calculate_theta(rotation);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, theta: %f", translation.x, translation.y, theta);
    }

    double calculate_theta(geometry_msgs::msg::Quaternion rotation)
    {
        double theta = tf2::getYaw(rotation);
        return theta;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotChase>());
    rclcpp::shutdown();
    return 0;
}