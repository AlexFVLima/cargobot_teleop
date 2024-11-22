// #include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CargobotTeleop : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    geometry_msgs::msg::Twist cmd_vel_msg_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    cmd_vel_msg_ = geometry_msgs::msg::Twist();

    if (!msg->buttons[1]) 
    {
        cmd_vel_msg_.linear.x = 10.0 * msg->axes[1];
        cmd_vel_msg_.angular.z = 10.0 * msg->axes[3];
    }
    else
    {
        cmd_vel_msg_.linear.x = 0;
        cmd_vel_msg_.angular.z = 0;
    }

    cmd_vel_pub_->publish(cmd_vel_msg_);
}

public:
    CargobotTeleop() : Node("cargobot_teleop")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&CargobotTeleop::joy_callback, this, std::placeholders::_1)
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared<>
    rclcpp::spin(std::make_shared<CargobotTeleop>());
    rclcpp::shutdown();
    return 0;
}
