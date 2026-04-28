#ifndef TIBBLE_CONTROLLER_HPP
#define TIBBLE_CONTROLLER_HPP

#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tibble_controller
{
    class TibbleController : public controller_interface::ControllerInterface
    {
        public:
            TibbleController() = default;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;


            controller_interface::CallbackReturn on_init() override;

            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
        

        private:
            // URDF joint names
            const std::string left_wheel_name_ = "Left Kraken";
            const std::string right_wheel_name_ = "Right Kraken";
            const std::string linear_actuator_name_ = "BLA Pivot";
            const std::string excavation_name_ = "E Roller";
            const std::string vibe_gpio_name_ = "vibe_control";
            const std::string latch_gpio_name_ = "latch_control";

            // Subs
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            // Pubs/Broadcasters
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            // Realtime buffers
            realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> twist_cmd_buffer_;
            realtime_tools::RealtimeBuffer<sensor_msgs::msg::Joy> joy_cmd_buffer_;

            // Params for kinematics (from tibble_controller.yaml)
            double wheel_radius_;
            double wheel_separation_;
            double paddle_speed_;
            double manual_paddle_max_speed_;
            double manual_la_speed_;

            // State machine enum
            enum class TibbleState {
                IDLE,
                TRAVEL,
                EXCAVATE,
                DUMP
            };
            TibbleState current_state_ = TibbleState::IDLE;
            TibbleState previous_state_ = TibbleState::IDLE;

            bool manual_mode_ = false;

            // LA target positions (in meters)
            const double LA_REST_POS = 0.00762; // 0.3 inches
            const double LA_EXCAV_POS = 0.0;
            const double LA_DUMP_POS = 0.07112; // 2.8 inches

            // Timers/timer values for sequential state management
            double state_timer_ = 0.0;
            const double EXCAVATE_PADDLE_DELAY = 1.0; // Wait before starting paddles
            const double DUMP_LA_DELAY = 0.2;      // Wait after unlatching before lifting
            const double DUMP_VIBE_DELAY = 1.0;    // Wait for LA to extend before vibrating

            // Button scheme
            const int STATE_TRAVEL_B = 7;   // 8
            const int STATE_IDLE_B = 6;     // 7
            const int STATE_EXCAVATE_B = 8; // 9
            const int STATE_DUMP_B = 9;     // 10

            // Manual mode tracking variables
            double manual_la_pos_ = LA_REST_POS;
            bool latch_state_ = true;  // true = latched
            bool vibe_state_ = false;  // false = off

            // Edge detection for toggles
            bool prev_manual_toggle_b_ = false;
            bool prev_latch_toggle_b_ = false;
            bool prev_vibe_toggle_b_ = false;

            // Manual Mode Button/Axis scheme placeholders
            const int MANUAL_TOGGLE_B = 10;       // 11
            const int MANUAL_LA_EXTEND_B = 6;     // 7
            const int MANUAL_LA_RETRACT_B = 7;    // 8
            const int MANUAL_LATCH_TOGGLE_B = 8;  // 9
            const int MANUAL_VIBE_TOGGLE_B = 9;   // 10
            const int MANUAL_EXCAV_AXIS = 3;      // Throttle

            // Odometry vars
            double odom_x_ = 0.0;
            double odom_y_ = 0.0;
            double odom_theta_ = 0.0;
            double last_left_wheel_pos_ = 0.0;
            double last_right_wheel_pos_ = 0.0;
            bool first_update_ = true;
    };
} // namespace tibble_controller

#endif // TIBBLE_CONTROLLER_HPP
