#include "control/tibble_controller.hpp"

#include <algorithm>
#include <optional>

#include <pluginlib/class_list_macros.hpp>

namespace tibble_controller
{
    controller_interface::InterfaceConfiguration TibbleController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            // claim joints with a command interface
        };

        return config;
    }


    controller_interface::InterfaceConfiguration TibbleController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            // claim joints with a state interface
        };

        return config;
    }


    controller_interface::CallbackReturn TibbleController::on_init()
    {
        try
        {
            // Pass params to nodes
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TibbleController::on_configure(const rclcpp_lifecycle::State &)
    {
        // Set local variables to the node parameter values

        // Initialize the realtime buffers if you have any

        // ROS subscriptions
        
        // ROS publishers

        // Do other things like set modes or fault latched state

        RCLCPP_INFO(get_node()->get_logger(), "Configured TibbleController.");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TibbleController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Make sure that the command interfaces are present

        // Make sure that the state interfaces are present

        // Safety outputs

        RCLCPP_INFO(get_node()->get_logger(), "Activated TibbleController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
   
   
    controller_interface::CallbackReturn TibbleController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop any sort of command outputs
        // If you're using a state machine, set mode to IDLE

        RCLCPP_INFO(get_node()->get_logger(), "Deactivated TibbleController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type TibbleController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Stuff goes here
        // Generally, first read states, second calculate kinematics, third write commands
    }

} // namespace tibble_controller


PLUGINLIB_EXPORT_CLASS(tibble_controller::TibbleController,
                       controller_interface::ControllerInterface)
