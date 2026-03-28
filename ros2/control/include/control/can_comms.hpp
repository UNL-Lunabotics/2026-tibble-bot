#ifndef TIBBLE_CAN_COMMS_HPP
#define TIBBLE_CAN_COMMS_HPP

#include <string>
#include <memory>
#include <cmath>   // Added for M_PI
#include <cstdlib> // Added for setenv

// CTRE Phoenix 6 Headers 
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

namespace tibble_hwc
{
    using namespace ctre::phoenix6;

    class CanComms
    {
    public:
        CanComms() = default;

        inline void setup(const std::string &can_interface, int left_id, int right_id) {
            // This MUST be called before the TalonFX objects are instantiated,
            // otherwise the internal diagnostic server defaults to "sim" mode.
            setenv("CTR_TARGET", "Hardware", 1);

            can_interface_ = can_interface;
            
            left_talon_ = std::make_shared<hardware::TalonFX>(left_id, can_interface_);
            right_talon_ = std::make_shared<hardware::TalonFX>(right_id, can_interface_);

            configs::TalonFXConfiguration left_cfg{};
            configs::TalonFXConfiguration right_cfg{};

            // Brake mode prevents the robot from rolling when commanded to 0
            left_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
            right_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

            // One side of the drivetrain must be inverted so that commanding 
            // a positive velocity moves both sides "forward".
            right_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive; 

            // VelocityVoltage is a closed-loop control. If kP and kV are 0, 
            // the motor will not output any voltage.
            left_cfg.Slot0.kP = 0.11; // Placeholder: Tune for your mass/gearing
            left_cfg.Slot0.kV = 0.12; // Placeholder: Feedforward
            right_cfg.Slot0.kP = 0.11;
            right_cfg.Slot0.kV = 0.12;

            left_talon_->GetConfigurator().Apply(left_cfg);
            right_talon_->GetConfigurator().Apply(right_cfg);
        }

        inline void send_velocities(double left_rad_s, double right_rad_s) {
            // On Linux, the Krakens will safety-disable if they don't receive 
            // a periodic enable signal. This keeps the motors active for 100ms.
            ctre::phoenix::unmanaged::FeedEnable(100); 

            double left_rps = left_rad_s / (2.0 * M_PI);
            double right_rps = right_rad_s / (2.0 * M_PI);

            // Use the specific units requested by the Phoenix 6 C++ API
            controls::VelocityVoltage left_cmd{units::angular_velocity::turns_per_second_t(left_rps)};
            controls::VelocityVoltage right_cmd{units::angular_velocity::turns_per_second_t(right_rps)};

            left_talon_->SetControl(left_cmd);
            right_talon_->SetControl(right_cmd);
        }

        inline double get_left_pos() {
            // GetPosition() returns the last cached value. To ensure ROS2 gets 
            // fresh odometry data off the CAN bus, call Refresh() first.
            return left_talon_->GetPosition().GetValue().value() * 2.0 * M_PI;
        }

        inline double get_right_pos() {
            return right_talon_->GetPosition().GetValue().value() * 2.0 * M_PI;
        }

        inline double get_left_vel() {
            return left_talon_->GetVelocity().GetValue().value() * 2.0 * M_PI;
        }

        inline double get_right_vel() {
            return right_talon_->GetVelocity().GetValue().value() * 2.0 * M_PI;
        }

    private:
        std::string can_interface_;
        std::shared_ptr<hardware::TalonFX> left_talon_;
        std::shared_ptr<hardware::TalonFX> right_talon_;
    };

} // namespace tibble_hwc

#endif