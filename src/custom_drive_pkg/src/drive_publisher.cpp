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

/**
 * DrivePublisher Node
 * 
 * Converts relative movement commands to absolute /map frame goals
 * Automatically cancels any previous navigation before sending new goal
 * 
 * Parameters:
 *   - forward: Distance to move forward in meters (negative = backward)
 *   - rotate: Angle to rotate in degrees (positive = counter-clockwise)
 *   - base_frame: Base frame name (default: base_link)
 * 
 * Example usage:
 *   # Move forward 2 meters
 *   ros2 run custom_drive_pkg drive_publisher --ros-args -p forward:=2.0 -p rotate:=0.0
 * 
 *   # Turn right 90 degrees (clockwise)
 *   ros2 run custom_drive_pkg drive_publisher --ros-args -p forward:=0.0 -p rotate:=-90.0
 * 
 *   # Move forward 1.5m and turn left 45 degrees
 *   ros2 run custom_drive_pkg drive_publisher --ros-args -p forward:=1.5 -p rotate:=45.0
 */
class DrivePublisher : public rclcpp::Node
{
public:
    DrivePublisher()
    : Node("drive_publisher"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      retry_count_(0),
      max_retries_(20)
    {
        // Declare parameters with defaults
        this->declare_parameter<double>("forward", 0.0);
        this->declare_parameter<double>("rotate", 0.0);
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<bool>("cancel_previous", true);  // Auto-cancel previous navigation
        
        // Get parameters
        forward_ = this->get_parameter("forward").as_double();
        rotate_deg_ = this->get_parameter("rotate").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        cancel_previous_ = this->get_parameter("cancel_previous").as_bool();
        
        // Convert rotation to radians
        rotate_rad_ = rotate_deg_ * M_PI / 180.0;
        
        // Create publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        // Retry every 500ms, up to 20 times (5 seconds total)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DrivePublisher::publish_goal_pose, this));
        
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║      Drive Publisher Node Started                 ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "  Forward:        %.2f m", forward_);
        RCLCPP_INFO(this->get_logger(), "  Rotate:         %.2f degrees (%.3f radians)", rotate_deg_, rotate_rad_);
        RCLCPP_INFO(this->get_logger(), "  Base frame:     %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Cancel previous: %s", cancel_previous_ ? "Yes" : "No");
        RCLCPP_INFO(this->get_logger(), "Waiting for TF transform...");
    }

private:
    /**
     * Get current robot pose in /map frame
     */
    bool get_current_pose(geometry_msgs::msg::PoseStamped& current_pose)
    {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "map", 
                base_frame_, 
                tf2::TimePointZero);
            
            current_pose.header.stamp = this->get_clock()->now();
            current_pose.header.frame_id = "map";
            current_pose.pose.position.x = transform_stamped.transform.translation.x;
            current_pose.pose.position.y = transform_stamped.transform.translation.y;
            current_pose.pose.position.z = transform_stamped.transform.translation.z;
            current_pose.pose.orientation = transform_stamped.transform.rotation;
            
            return true;
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform (attempt %d/%d): %s", 
                        retry_count_ + 1, max_retries_, ex.what());
            return false;
        }
    }
    
    /**
     * Extract yaw angle from quaternion
     */
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quat, tf_quat);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        
        return yaw;
    }
    
    /**
     * Create quaternion from yaw angle
     */
    geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw)
    {
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0, 0, yaw);
        tf_quat.normalize();
        
        return tf2::toMsg(tf_quat);
    }
    
    /**
     * Cancel any previous navigation by publishing current position as goal
     */
    void cancel_previous_navigation()
    {
        if (!cancel_previous_) {
            return;  // Feature disabled
        }
        
        RCLCPP_INFO(this->get_logger(), "→ Canceling any previous navigation...");
        
        geometry_msgs::msg::PoseStamped current_pose;
        if (get_current_pose(current_pose)) {
            // Publish current position as goal - tells Nav2 "I'm already here"
            publisher_->publish(current_pose);
            
            // Wait for Nav2 to process the cancellation
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            
            RCLCPP_INFO(this->get_logger(), "→ Previous navigation canceled");
        } else {
            RCLCPP_WARN(this->get_logger(), "→ Could not cancel previous navigation (no current pose)");
        }
    }
    
    /**
     * Convert relative movement to absolute goal pose and publish
     */
    void publish_goal_pose()
    {
        geometry_msgs::msg::PoseStamped current_pose;
        
        // Try to get current pose
        if (!get_current_pose(current_pose)) {
            retry_count_++;
            
            if (retry_count_ >= max_retries_) {
                RCLCPP_ERROR(this->get_logger(), 
                    "╔═══════════════════════════════════════════════════╗");
                RCLCPP_ERROR(this->get_logger(), 
                    "║  Failed to get transform after %d attempts!       ║", max_retries_);
                RCLCPP_ERROR(this->get_logger(), 
                    "║  Make sure SLAM and Leo base system are running! ║");
                RCLCPP_ERROR(this->get_logger(), 
                    "╚═══════════════════════════════════════════════════╝");
                rclcpp::shutdown();
            }
            return;  // Will retry on next timer callback
        }
        
        // Success! Stop the timer
        timer_->cancel();
        
        // Log current position
        double current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "✓ Transform found!");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Current pose in /map frame:");
        RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)", 
                    current_pose.pose.position.x, 
                    current_pose.pose.position.y, 
                    current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Yaw: %.2f degrees", current_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "");
        
        // Cancel any previous navigation
        cancel_previous_navigation();
        
        // Create goal pose
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        
        // Calculate new position based on current orientation and forward distance
        // Robot moves forward along its current heading
        double new_x = current_pose.pose.position.x + forward_ * cos(current_yaw);
        double new_y = current_pose.pose.position.y + forward_ * sin(current_yaw);
        
        goal_pose.pose.position.x = new_x;
        goal_pose.pose.position.y = new_y;
        goal_pose.pose.position.z = current_pose.pose.position.z;  // Keep same height
        
        // Calculate new orientation (add rotation to current yaw)
        double new_yaw = current_yaw + rotate_rad_;
        goal_pose.pose.orientation = create_quaternion_from_yaw(new_yaw);
        
        // Publish goal
        publisher_->publish(goal_pose);
        
        // Log goal position
        RCLCPP_INFO(this->get_logger(), "╔═══════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║           Goal Published to Nav2                  ║");
        RCLCPP_INFO(this->get_logger(), "╚═══════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "Goal pose in /map frame:");
        RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)", 
                    goal_pose.pose.position.x, 
                    goal_pose.pose.position.y, 
                    goal_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Yaw: %.2f degrees", new_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Movement command:");
        RCLCPP_INFO(this->get_logger(), "  Delta distance: (%.3f, %.3f) m", 
                    new_x - current_pose.pose.position.x,
                    new_y - current_pose.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "  Delta rotation: %.2f degrees", rotate_deg_);
        RCLCPP_INFO(this->get_logger(), "");
        
        // Shutdown after publishing
        RCLCPP_INFO(this->get_logger(), "Node shutting down. Nav2 is now executing the goal.");
        rclcpp::shutdown();
    }
    
    // ROS2 objects
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Parameters
    double forward_;         // Distance to move forward (meters)
    double rotate_deg_;      // Rotation angle (degrees)
    double rotate_rad_;      // Rotation angle (radians)
    std::string base_frame_; // Base frame name
    bool cancel_previous_;   // Auto-cancel previous navigation
    int retry_count_;        // Current retry attempt
    int max_retries_;        // Maximum retry attempts
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrivePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

