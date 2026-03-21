#ifndef TIBBLE_CONTROLLER_HPP
#define TIBBLE_CONTROLLER_HPP

#include

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
            // Any necessary structs, helper functions, variables, etc
            // Realtime tools go here
            // ROS subs and pubs go here
            // Params from the URDF go here
            // Specifically, put std::string joint_names{"joint_name"} here for all joints
    };
} // namespace tibble_controller

#endif // TIBBLE_CONTROLLER_HPP
