#ifndef TIBBLE_HWC_HPP
#define TIBBLE_HWC_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Communication layers
#include "control/arduino_comms.hpp"
// #include "control/can_comms.hpp"

namespace tibble_hwc
{
    class TibbleHWC : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(TibbleHWC)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareComponentInterfaceParams & params) override;
            
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
        private:
            // --- Communication Objects ---
            ArduinoComms teensy_comms_;
            CanComms can_comms_;

            // --- Hardware Parameters (Pulled from URDF) ---
            std::string serial_port_;
            int baud_rate_;
            std::string can_interface_;

            // Firgelli 450lb Super Duty: 41.1 pulses/mm
            const double LA_TICKS_PER_METER = 41100.0;
            const double LA_SYNC_TOLERANCE_METERS = 0.02; // Throw error if > 20mm out of sync
            
            // goBILDA 5203 (117 RPM)
            const double EXCAV_MAX_RAD_S = 12.25;
            const int ROBOCLAW_MAX_PWM = 127;

            // --- Internal Joint Memory ---
            
            // Drivetrain (CAN)
            double cmd_left_wheel_vel_{0.0};
            double cmd_right_wheel_vel_{0.0};
            double state_left_wheel_pos_{0.0};
            double state_left_wheel_vel_{0.0};
            double state_right_wheel_pos_{0.0};
            double state_right_wheel_vel_{0.0};

            // Linear Actuators (Serial)
            double cmd_la_pos_{0.0};
            double cmd_la_reset_{0.0};   // 1.0 triggers an encoder zeroing
            double state_la_pos_{0.0};   // Averaged state of both LAs
            
            // Mechanisms (Serial)
            double cmd_excav_vel_{0.0};
            double cmd_vibe_enabled_{0.0};
            double cmd_hop_latched_{1.0}; // Default to latched (1.0)
            
            // Raw internal tracking
            int32_t raw_la_1_ticks_{0};
            int32_t raw_la_2_ticks_{0};
            int32_t raw_excav_ticks_{0};
    };
}   // namespace tibble_hwc

#endif // TIBBLE_HWC_HPP