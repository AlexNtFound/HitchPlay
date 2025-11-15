#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "custom_drive_pkg/srv/drive_command.hpp"

class DriveService : public rclcpp::Node
{
public:
    DriveService()
    : Node("drive_service"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        this->declare_parameter<std::string>("base_frame", "base_link");
        base_frame_ = this->get_parameter("base_frame").as_string();
        
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        drive_service_ = this->create_service<custom_drive_pkg::srv::DriveCommand>(
            "drive_command",
            std::bind(&DriveService::handle_drive, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║   Drive Service Ready                  ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "Service: /drive_command");
        RCLCPP_INFO(this->get_logger(), "Usage: ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand \"{forward: 1.0, rotate: 0.0}\"");
    }

private:
    bool get_current_pose(geometry_msgs::msg::PoseStamped& pose)
    {
        try {
            auto transform = tf_buffer_->lookupTransform("map", base_frame_, tf2::TimePointZero);
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        }
        catch (tf2::TransformException &ex) {
            return false;
        }
    }
    
    double get_yaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion quat;
        tf2::fromMsg(q, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();
        return tf2::toMsg(q);
    }
    
    void handle_drive(
        const std::shared_ptr<custom_drive_pkg::srv::DriveCommand::Request> request,
        std::shared_ptr<custom_drive_pkg::srv::DriveCommand::Response> response)
    {
        double forward = request->forward;
        double rotate_deg = request->rotate;
        double rotate_rad = rotate_deg * M_PI / 180.0;
        
        RCLCPP_INFO(this->get_logger(), "→ Command: forward=%.2fm, rotate=%.1f°", forward, rotate_deg);
        
        geometry_msgs::msg::PoseStamped current;
        if (!get_current_pose(current)) {
            response->success = false;
            response->message = "Failed to get pose - check SLAM/TF";
            return;
        }
        
        double yaw = get_yaw(current.pose.orientation);
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = this->get_clock()->now();
        goal.pose.position.x = current.pose.position.x + forward * cos(yaw);
        goal.pose.position.y = current.pose.position.y + forward * sin(yaw);
        goal.pose.position.z = current.pose.position.z;
        goal.pose.orientation = yaw_to_quat(yaw + rotate_rad);
        
        // Cancel previous
        goal_pub_->publish(current);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Publish goal
        goal_pub_->publish(goal);
        
        RCLCPP_INFO(this->get_logger(), "✓ Goal: (%.2f, %.2f) yaw: %.1f°", 
                    goal.pose.position.x, goal.pose.position.y, 
                    (yaw + rotate_rad) * 180.0 / M_PI);
        
        response->success = true;
        response->message = "Goal published";
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Service<custom_drive_pkg::srv::DriveCommand>::SharedPtr drive_service_;
    std::string base_frame_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveService>());
    rclcpp::shutdown();
    return 0;
}

