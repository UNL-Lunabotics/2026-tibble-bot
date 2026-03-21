#ifndef TIBBLE_CONTROLLER_HPP
#define TIBBLE_CONTROLLER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "realtime_tools/realtime_buffer.h"

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
            const std::string left_wheel_name_ = "DL_Rotate";
            const std::string right_wheel_name_ = "DR_Rotate";
            const std::string linear_actuator_name_ = "LA_Extend";
            const std::string excavation_name_ = "Excavation_Rotate";
            const std::string vibe_gpio_name_ = "vibe_control";
            const std::string latch_gpio_name_ = "latch_control";

            // Subs
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            // Realtime buffers
            realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> twist_cmd_buffer_;
            realtime_tools::RealtimeBuffer<sensor_msgs::msg::Joy> joy_cmd_buffer_;

            // Params for kinematics (from tibble_controller.yaml)
            double wheel_radius_;
            double wheel_separation_;
            double paddle_speed_;

            // State machine enum
            enum class TibbleState {
                IDLE,
                TRAVEL,
                EXCAVATE,
                DUMP
            };
            TibbleState current_state_ = TibbleState::IDLE;
            TibbleState previous_state_ = TibbleState::IDLE;

            // LA target positions (in meters)
            const double LA_REST_POS = 0.0; // not true, fix later TODO
            const double LA_EXCAV_POS = 0.0;
            const double LA_DUMP_POS = 0.254; // 10 inch

            // Timers/timer values for sequential state management
            double state_timer_ = 0.0;
            const double EXCAVATE_PADDLE_DELAY = 1.0; // Wait before starting paddles
            const double DUMP_LA_DELAY = 0.2;      // Wait after unlatching before lifting
            const double DUMP_VIBE_DELAY = 1.0;    // Wait for LA to extend before vibrating

            // Button scheme
            const int STATE_TRAVEL_B = 0;
            const int STATE_IDLE_B = 0;
            const int STATE_EXCAVATE_B = 0;
            const int STATE_DUMP_B = 0;

    };
} // namespace tibble_controller

#endif // TIBBLE_CONTROLLER_HPP
