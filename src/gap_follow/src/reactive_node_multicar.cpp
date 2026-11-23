#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Parameterized Follow The Gap Controller for Multi-Vehicle ROS 2 Systems.
 * 
 * Supports independent topic mapping, namespaces, and runtime parameter configuration
 * allowing multiple vehicles (e.g. f1tenth_1, f1tenth_2) to execute the exact same
 * reactive obstacle-avoidance algorithm simultaneously without topic collisions.
 */
class ReactiveFollowGapMultiCar : public rclcpp::Node {
public:
    ReactiveFollowGapMultiCar() : Node("reactive_node") {
        // Declare configurable ROS 2 parameters
        this->declare_parameter<std::string>("car_id", "f1tenth_1");
        this->declare_parameter<std::string>("lidar_topic", "/autodrive/f1tenth_1/lidar");
        this->declare_parameter<std::string>("throttle_topic", "/autodrive/f1tenth_1/throttle_command");
        this->declare_parameter<std::string>("steering_topic", "/autodrive/f1tenth_1/steering_command");
        this->declare_parameter<std::string>("processed_scan_topic", "/f1tenth_1/processed_scan");
        this->declare_parameter<std::string>("marker_topic", "/f1tenth_1/visualization_marker");

        this->declare_parameter<double>("view_angle_deg", 70.0);
        this->declare_parameter<double>("max_lidar_range", 10.0);
        this->declare_parameter<double>("min_obstacle_dist", 0.10);
        this->declare_parameter<int>("bubble_radius", 18);
        this->declare_parameter<double>("gap_threshold", 1.20);
        this->declare_parameter<double>("straight_throttle", 0.25);
        this->declare_parameter<double>("turning_throttle", 0.12);
        this->declare_parameter<double>("straight_angle_threshold", 0.10);

        // Retrieve parameter values
        car_id_ = this->get_parameter("car_id").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string throttle_topic = this->get_parameter("throttle_topic").as_string();
        std::string steering_topic = this->get_parameter("steering_topic").as_string();
        std::string processed_scan_topic = this->get_parameter("processed_scan_topic").as_string();
        std::string marker_topic = this->get_parameter("marker_topic").as_string();

        view_angle_deg_ = this->get_parameter("view_angle_deg").as_double();
        max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
        min_obstacle_dist_ = this->get_parameter("min_obstacle_dist").as_double();
        bubble_radius_ = this->get_parameter("bubble_radius").as_int();
        gap_threshold_ = this->get_parameter("gap_threshold").as_double();
        straight_throttle_ = this->get_parameter("straight_throttle").as_double();
        turning_throttle_ = this->get_parameter("turning_throttle").as_double();
        straight_angle_thresh_ = this->get_parameter("straight_angle_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing Follow The Gap for [%s]", car_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  LiDAR Topic:    %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Throttle Topic: %s", throttle_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Steering Topic: %s", steering_topic.c_str());

        // Control publishers
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(throttle_topic, 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(steering_topic, 10);

        // Diagnostics publishers
        processed_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(processed_scan_topic, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

        // LiDAR subscription
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10,
            std::bind(&ReactiveFollowGapMultiCar::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        auto ranges = scan_msg->ranges;
        int size = static_cast<int>(ranges.size());
        if (size == 0) return;

        // 1. Frontal Field of View (+/- view_angle_deg_)
        float view_angle_rad = static_cast<float>(view_angle_deg_ * M_PI / 180.0);
        int min_idx = static_cast<int>(((-view_angle_rad) - scan_msg->angle_min) / scan_msg->angle_increment);
        int max_idx = static_cast<int>(((view_angle_rad) - scan_msg->angle_min) / scan_msg->angle_increment);

        min_idx = std::max(0, min_idx);
        max_idx = std::min(size - 1, max_idx);

        // 2. Preprocessing
        for (int i = 0; i < size; ++i) {
            if (!std::isfinite(ranges[i]) || i < min_idx || i > max_idx) {
                ranges[i] = 0.0f;
            }
            if (ranges[i] > max_lidar_range_) {
                ranges[i] = static_cast<float>(max_lidar_range_);
            }
        }

        // 3. Find closest obstacle inside the frontal sector
        int closest_index = min_idx;
        float min_dist = static_cast<float>(max_lidar_range_);
        for (int i = min_idx; i <= max_idx; ++i) {
            if (ranges[i] > min_obstacle_dist_ && ranges[i] < min_dist) {
                min_dist = ranges[i];
                closest_index = i;
            }
        }

        // 4. Create Dynamic Safety Bubble around the closest obstacle
        // Increased half-width safety margin to 0.35m to prevent side-clipping
        float bubble_angle = std::atan2(0.35f, std::max(0.20f, min_dist));
        int effective_bubble = std::max(bubble_radius_, static_cast<int>(bubble_angle / scan_msg->angle_increment));
        for (int i = -effective_bubble; i <= effective_bubble; ++i) {
            int idx = closest_index + i;
            if (idx >= 0 && idx < size) {
                ranges[idx] = 0.0f;
            }
        }

        // 5. Find Largest Free-Space Gap
        int max_start = min_idx;
        int max_end = min_idx;
        int current_start = min_idx;
        bool in_gap = false;

        for (int i = min_idx; i <= max_idx; ++i) {
            if (ranges[i] > gap_threshold_) {
                if (!in_gap) {
                    current_start = i;
                    in_gap = true;
                }
            } else {
                if (in_gap) {
                    if ((i - current_start) > (max_end - max_start)) {
                        max_start = current_start;
                        max_end = i;
                    }
                    in_gap = false;
                }
            }
        }

        if (in_gap && (max_idx - current_start) > (max_end - max_start)) {
            max_start = current_start;
            max_end = max_idx;
        }

        // 6. Select Steering Target: Center of largest gap
        int best_index = (min_idx + max_idx) / 2;
        if (max_end > max_start) {
            best_index = (max_start + max_end) / 2;
        } else {
            // Fallback: If no contiguous gap exceeds gap_threshold (e.g. sharp turn),
            // steer toward the beam with maximum clearance in the frontal FOV.
            float max_val = -1.0f;
            for (int i = min_idx; i <= max_idx; ++i) {
                if (ranges[i] > max_val) {
                    max_val = ranges[i];
                    best_index = i;
                }
            }
        }

        float angle = scan_msg->angle_min + best_index * scan_msg->angle_increment;
        // Clamp to normalized steering limits [-1.0, +1.0] for AutoDRIVE simulator
        angle = std::max(-1.0f, std::min(1.0f, angle));

        // 7. Dynamic Throttle Control with Multi-Car Collision Prevention
        std_msgs::msg::Float32 throttle_msg;
        std_msgs::msg::Float32 steering_msg;
        steering_msg.data = angle;

        float base_speed = (std::abs(angle) < straight_angle_thresh_)
            ? static_cast<float>(straight_throttle_)
            : static_cast<float>(turning_throttle_);

        // Collision avoidance safety buffer:
        // Automatically paces following distance if another vehicle is in frontal sector
        if (min_dist < 0.85f) {
            // Safe following brake: zero throttle when too close to lead vehicle or obstacle
            throttle_msg.data = 0.0f;
        } else if (min_dist < 1.30f) {
            // Pacing following speed (40% throttle)
            throttle_msg.data = base_speed * 0.40f;
        } else if (min_dist < 1.80f) {
            // Approaching buffer (70% throttle)
            throttle_msg.data = base_speed * 0.70f;
        } else {
            // Clear path: full speed ahead
            throttle_msg.data = base_speed;
        }

        throttle_pub_->publish(throttle_msg);
        steering_pub_->publish(steering_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[%s] Active Follow-The-Gap | MinDist: %.2fm | Throttle: %.2f | Steer: %.2f rad",
            car_id_.c_str(), min_dist, throttle_msg.data, steering_msg.data);

        // Publish Processed Scan
        auto new_scan = *scan_msg;
        new_scan.ranges = ranges;
        processed_scan_pub_->publish(new_scan);

        // Publish Target Direction Arrow Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = scan_msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = car_id_ + "_direction";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.z = std::sin(angle / 2.0f);
        marker.pose.orientation.w = std::cos(angle / 2.0f);
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0f;
        if (car_id_ == "f1tenth_2") {
            marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f; // Orange
        } else {
            marker.color.r = 0.0f; marker.color.g = 0.9f; marker.color.b = 1.0f; // Cyan
        }

        marker_pub_->publish(marker);
    }

    std::string car_id_;
    double view_angle_deg_;
    double max_lidar_range_;
    double min_obstacle_dist_;
    int bubble_radius_;
    double gap_threshold_;
    double straight_throttle_;
    double turning_throttle_;
    double straight_angle_thresh_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGapMultiCar>());
    rclcpp::shutdown();
    return 0;
}
