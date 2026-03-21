#include "control/tibble_hwc.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tibble_hwc
{
    hardware_interface::CallbackReturn TibbleHWC::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        serial_port_ = info_.hardware_parameters["serial_port"];
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        can_interface_ = info_.hardware_parameters["can_interface"];

        teensy_comms_.setup(serial_port_, baud_rate_, 100);
        can_comms_.setup(can_interface_, 1, 2); // placeholder IDs

        RCLCPP_INFO(rclcpp::get_logger("TibbleHWC"), "Initialized with Serial: %s and CAN: %s", serial_port_.c_str(), can_interface_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn TibbleHWC::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TibbleHWC"), "Configuring hardware...");
        teensy_comms_.connect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn TibbleHWC::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TibbleHWC"), "Cleaning up hardware...");
        teensy_comms_.disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn TibbleHWC::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TibbleHWC"), "Activating hardware interfaces...");
        
        // Reset internal memory to safe defaults
        cmd_left_wheel_vel_ = 0.0;
        cmd_right_wheel_vel_ = 0.0;
        cmd_la_pos_ = 0.0;
        cmd_la_reset_ = 0.0;
        cmd_excav_vel_ = 0.0;
        cmd_vibe_enabled_ = 0.0;
        cmd_hop_latched_ = 1.0;

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn TibbleHWC::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("TibbleHWC"), "Deactivating. Sending Emergency Stop.");
        teensy_comms_.send_stop_command();
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> TibbleHWC::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Drivetrain
        state_interfaces.emplace_back(hardware_interface::StateInterface("DL_Rotate", hardware_interface::HW_IF_POSITION, &state_left_wheel_pos_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("DL_Rotate", hardware_interface::HW_IF_VELOCITY, &state_left_wheel_vel_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("DR_Rotate", hardware_interface::HW_IF_POSITION, &state_right_wheel_pos_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("DR_Rotate", hardware_interface::HW_IF_VELOCITY, &state_right_wheel_vel_));
        
        // Linear Actuator
        state_interfaces.emplace_back(hardware_interface::StateInterface("LA_Extend", hardware_interface::HW_IF_POSITION, &state_la_pos_));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> TibbleHWC::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface("DL_Rotate", hardware_interface::HW_IF_VELOCITY, &cmd_left_wheel_vel_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("DR_Rotate", hardware_interface::HW_IF_VELOCITY, &cmd_right_wheel_vel_));
        
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LA_Extend", hardware_interface::HW_IF_POSITION, &cmd_la_pos_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LA_Extend", "la_reset", &cmd_la_reset_));
        
        command_interfaces.emplace_back(hardware_interface::CommandInterface("Excavation_Rotate", hardware_interface::HW_IF_VELOCITY, &cmd_excav_vel_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("vibe_port", "vibe_enabled", &cmd_vibe_enabled_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("latch_port", "hop_latched", &cmd_hop_latched_));

        return command_interfaces;
    }


    hardware_interface::return_type TibbleHWC::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        teensy_comms_.read_encoder_values(raw_la_1_ticks_, raw_la_2_ticks_, raw_excav_ticks_);

        // 2. Linear Actuator Sync Check
        double la_1_meters = raw_la_1_ticks_ / LA_TICKS_PER_METER;
        double la_2_meters = raw_la_2_ticks_ / LA_TICKS_PER_METER;
        
        if (std::abs(la_1_meters - la_2_meters) > LA_SYNC_TOLERANCE_METERS) {
            RCLCPP_FATAL(rclcpp::get_logger("TibbleHWC"), "CRITICAL FAULT: Linear Actuators out of sync by > 20mm! Halting system.");
            return hardware_interface::return_type::ERROR; 
        }

        // 3. Average the LAs for the ROS Controller State
        state_la_pos_ = (la_1_meters + la_2_meters) / 2.0;

        // 4. Read from CAN bus for wheels
        state_left_wheel_pos_ = can_comms_.get_left_pos();
        state_right_wheel_pos_ = can_comms_.get_right_pos();
        state_left_wheel_vel_ = can_comms_.get_left_vel();
        state_right_wheel_vel_ = can_comms_.get_right_vel();

        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type TibbleHWC::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 1. Handle Encoder Reset Request
        if (cmd_la_reset_ >= 1.0) {
            teensy_comms_.send_reset_command(); // Reset linear encoders
            
            cmd_la_reset_ = 0.0; // Instantly consume the command
            return hardware_interface::return_type::OK; // Skip the rest of the loop for this tick
        }

        // 2. Linear Actuator Position -> PWM Conversion (P-Controller)
        double la_error_meters = cmd_la_pos_ - state_la_pos_;
        double kP = 2000.0; // Tuning parameter: higher = more aggressive approach
        
        // RoboClaw expects 64 = stop, 0 = full reverse, 127 = full forward
        int la_pwm = 64 + std::clamp(static_cast<int>(la_error_meters * kP), -63, 63);
        
        // Deadband: If we are within 1mm of the target, stop the motor to prevent jittering
        if (std::abs(la_error_meters) < 0.001) {
            la_pwm = 64; 
        }

        // 3. Excavation Rad/s -> PWM Conversion
        int excav_pwm = 64;
        if (cmd_excav_vel_ > 0.0) {
            // Map 0 -> 12.25 rad/s to 64 -> 127 PWM
            double excav_ratio = cmd_excav_vel_ / EXCAV_MAX_RAD_S;
            excav_pwm = 64 + std::clamp(static_cast<int>(excav_ratio * 63.0), 0, 63);
        }

        // 4. Vibe Pseudo-Boolean -> PWM Conversion
        int vibe_pwm = (cmd_vibe_enabled_ > 0.5) ? 127 : 64;

        // 5. Latch Pseudo-Boolean -> Servo Angle Conversion
        int latch_angle = (cmd_hop_latched_ > 0.5) ? 180 : 0;

        // 6. Send the formatted command to the Teensy
        // Assuming ArduinoComms handles formatting into "c %d %d %d %d %d\n"
        teensy_comms_.send_commands(la_pwm, la_pwm, vibe_pwm, excav_pwm, latch_angle);

        // 7. Write to Drivetrain via CAN (Placeholder)
        can_comms_.send_velocities(cmd_left_wheel_vel_, cmd_right_wheel_vel_);

        return hardware_interface::return_type::OK;
    }

} // namespace tibble_hwc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tibble_hwc::TibbleHWC, hardware_interface::SystemInterface)