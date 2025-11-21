#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "custom_drive_pkg/srv/drive_command.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class DriveService : public rclcpp::Node
{
public:
    DriveService()
    : Node("drive_service"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("goal_timeout", 300.0);
        base_frame_ = this->get_parameter("base_frame").as_string();
        goal_timeout_ = this->get_parameter("goal_timeout").as_double();
        
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        drive_service_ = this->create_service<custom_drive_pkg::srv::DriveCommand>(
            "drive_command",
            std::bind(&DriveService::handle_drive, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║   Sequential Drive Service Ready       ║");
        RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "Service: /drive_command");
        RCLCPP_INFO(this->get_logger(), "Supports infinite sequential commands");
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
            RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
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
    
    bool send_goal_and_wait(const geometry_msgs::msg::PoseStamped& goal)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return false;
        }
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal;
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Feedback callback (optional, for monitoring)
        send_goal_options.feedback_callback = 
            [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                // You can log feedback here if needed
            };
        
        // Send goal
        auto goal_handle_future = nav_client_->async_send_goal(goal_msg, send_goal_options);
        
        // Wait for goal acceptance
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future, 
                                               std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }
        
        // Wait for result
        auto result_future = nav_client_->async_get_result(goal_handle);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 
                                               std::chrono::seconds(static_cast<int>(goal_timeout_))) != 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal execution timeout");
            nav_client_->async_cancel_all_goals();
            return false;
        }
        
        auto result = result_future.get();
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
            return false;
        }
    }
    
    void handle_drive(
        const std::shared_ptr<custom_drive_pkg::srv::DriveCommand::Request> request,
        std::shared_ptr<custom_drive_pkg::srv::DriveCommand::Response> response)
    {
        int total_commands = request->commands.size();
        RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "📋 Received %d sequential commands", total_commands);
        RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
        
        if (total_commands == 0) {
            response->success = false;
            response->message = "No commands provided";
            response->completed_steps = 0;
            return;
        }
        
        int completed = 0;
        
        for (int i = 0; i < total_commands; i++) {
            const auto& cmd = request->commands[i];
            double forward = cmd.forward;
            double rotate_deg = cmd.rotate;
            double rotate_rad = rotate_deg * M_PI / 180.0;
            
            RCLCPP_INFO(this->get_logger(), "→ Step %d/%d: forward=%.2fm, rotate=%.1f°", 
                        i+1, total_commands, forward, rotate_deg);
            
            // Get current pose
            geometry_msgs::msg::PoseStamped current;
            if (!get_current_pose(current)) {
                response->success = false;
                response->message = "Failed to get pose at step " + std::to_string(i+1);
                response->completed_steps = completed;
                return;
            }
            
            double yaw = get_yaw(current.pose.orientation);
            
            // Calculate goal
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = this->get_clock()->now();
            goal.pose.position.x = current.pose.position.x + forward * cos(yaw);
            goal.pose.position.y = current.pose.position.y + forward * sin(yaw);
            goal.pose.position.z = current.pose.position.z;
            goal.pose.orientation = yaw_to_quat(yaw + rotate_rad);
            
            RCLCPP_INFO(this->get_logger(), "  Goal: (%.2f, %.2f) yaw: %.1f°", 
                        goal.pose.position.x, goal.pose.position.y, 
                        (yaw + rotate_rad) * 180.0 / M_PI);
            
            // Send goal and wait for completion
            if (!send_goal_and_wait(goal)) {
                response->success = false;
                response->message = "Failed at step " + std::to_string(i+1) + " of " + std::to_string(total_commands);
                response->completed_steps = completed;
                RCLCPP_ERROR(this->get_logger(), "✗ Step %d/%d FAILED", i+1, total_commands);
                return;
            }
            
            completed++;
            RCLCPP_INFO(this->get_logger(), "✓ Step %d/%d COMPLETED", i+1, total_commands);
        }
        
        response->success = true;
        response->message = "All " + std::to_string(total_commands) + " commands completed successfully";
        response->completed_steps = completed;
        
        RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "✓ ALL %d STEPS COMPLETED SUCCESSFULLY", total_commands);
        RCLCPP_INFO(this->get_logger(), "════════════════════════════════════════");
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Service<custom_drive_pkg::srv::DriveCommand>::SharedPtr drive_service_;
    std::string base_frame_;
    double goal_timeout_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveService>());
    rclcpp::shutdown();
    return 0;
}
