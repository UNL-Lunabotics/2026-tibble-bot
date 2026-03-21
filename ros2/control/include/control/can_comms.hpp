#ifndef TIBBLE_CAN_COMMS_HPP
#define TIBBLE_CAN_COMMS_HPP

#include <string>
#include <memory>

// CTRE Phoenix 6 Headers
// Note: You must install the phoenix6 apt package for this to compile!
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"

// This is AI generated slop I haven't fixed yet since idk the CAN situation
namespace tibble_hwc
{
    using namespace ctre::phoenix6;

    class CanComms
    {
    public:
        CanComms() = default;

        inline void setup(const std::string &can_interface, int left_id, int right_id) {
            can_interface_ = can_interface;
            
            // Initialize the TalonFX objects on the specified CAN bus
            left_talon_ = std::make_shared<hardware::TalonFX>(left_id, can_interface_);
            right_talon_ = std::make_shared<hardware::TalonFX>(right_id, can_interface_);

            // Apply basic configurations (Current limits, neutral mode, etc.)
            configs::TalonFXConfiguration cfg{};
            cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
            
            left_talon_->GetConfigurator().Apply(cfg);
            right_talon_->GetConfigurator().Apply(cfg);
        }

        inline void send_velocities(double left_rad_s, double right_rad_s) {
            // Kraken X60/Talon FX expects velocity in Rotations Per Second (RPS)
            // Convert rad/s to Rotations/s
            double left_rps = left_rad_s / (2.0 * M_PI);
            double right_rps = right_rad_s / (2.0 * M_PI);

            // Using VelocityVoltage control mode (requires PID gains set on the Talon)
            // If you haven't tuned PID yet, use DutyCycleOut for raw % power
            controls::VelocityVoltage left_cmd{units::angular_velocity::revolutions_per_second_t(left_rps)};
            controls::VelocityVoltage right_cmd{units::angular_velocity::revolutions_per_second_t(right_rps)};

            left_talon_->SetControl(left_cmd);
            right_talon_->SetControl(right_cmd);
        }

        inline double get_left_pos() {
            // Returns position in Radians
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