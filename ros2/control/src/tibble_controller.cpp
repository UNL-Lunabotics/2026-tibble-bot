#include "control/tibble_controller.hpp"

#include <algorithm>
#include <optional>

#include <pluginlib/class_list_macros.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace tibble_controller
{
    controller_interface::InterfaceConfiguration TibbleController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = { // order in here determines array index order
            left_wheel_name_ + "/velocity",
            right_wheel_name_ + "/velocity",
            linear_actuator_name_ + "/position",
            excavation_name_ + "/position",
            vibe_gpio_name_ + "/vibe_enabled",
            latch_gpio_name_ + "/hop_latched",
            linear_actuator_name_ + "/la_reset"
        };

        return config;
    }


    controller_interface::InterfaceConfiguration TibbleController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            left_wheel_name_ + "/velocity",
            left_wheel_name_ + "/position",
            right_wheel_name_ + "/velocity",
            right_wheel_name_ + "/position",
            linear_actuator_name_ + "/position",
            excavation_name_ + "/position"
        };

        return config;
    }


    controller_interface::CallbackReturn TibbleController::on_init()
    {
        try
        {
            // Receive params from tibble_controller.yaml
            auto_declare<double>("wheel_radius", 0.0);
            auto_declare<double>("wheel_separation", 0.0);
            auto_declare<double>("paddle_speed", 0.0);
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
        wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
        wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
        paddle_speed_ = get_node()->get_parameter("paddle_speed").as_double();

        // Initialize the realtime buffers if you have any

        // ROS subscriptions
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                twist_cmd_buffer_.writeFromNonRT(*msg);
            });
        
        joy_sub_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
            "~/joy", rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                joy_cmd_buffer_.writeFromNonRT(*msg);
            });
        
        // ROS publishers
        odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*get_node());

        RCLCPP_INFO(get_node()->get_logger(), "Configured TibbleController.");
        
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::CallbackReturn TibbleController::on_activate(const rclcpp_lifecycle::State &)
    {
        current_state_ = TibbleState::IDLE;

        twist_cmd_buffer_.reset();
        joy_cmd_buffer_.reset();

        // Odom reset
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        first_update_ = true;

        if (command_interfaces_.empty() || state_interfaces_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Missing controller interfaces.");
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Activated TibbleController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
   
   
    controller_interface::CallbackReturn TibbleController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        current_state_ = TibbleState::IDLE;

        RCLCPP_INFO(get_node()->get_logger(), "Deactivated TibbleController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::return_type TibbleController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time; // Silence unused param warning

        // --- First, read inputs ---
        double current_left_pos = state_interfaces_[0].get_optional().value_or(0.0);
        // Skip velocity states [1] and [3] for now
        double current_right_pos = state_interfaces_[2].get_optional().value_or(0.0);
        double current_la_pos = state_interfaces_[4].get_optional().value_or(0.0);


        // --- Second, odometry math ---
        if (first_update_) {
            last_left_wheel_pos_ = current_left_pos;
            last_right_wheel_pos_ = current_right_pos;
            first_update_ = false;
        }

        // Calculate how much the wheels moved in radians since last tick
        double delta_left = current_left_pos - last_left_wheel_pos_;
        double delta_right = current_right_pos - last_right_wheel_pos_;

        // Convert radians to linear distance (meters)
        double distance_left = delta_left * wheel_radius_;
        double distance_right = delta_right * wheel_radius_;

        // Calculate robot's translation and rotation
        double linear_distance = (distance_right + distance_left) / 2.0;
        double angular_distance = (distance_right - distance_left) / wheel_separation_;

        // Update global pose using Runge-Kutta 2nd order approximation
        odom_theta_ += angular_distance;
        odom_x_ += linear_distance * std::cos(odom_theta_ - angular_distance / 2.0);
        odom_y_ += linear_distance * std::sin(odom_theta_ - angular_distance / 2.0);

        // Save current positions for the next loop
        last_left_wheel_pos_ = current_left_pos;
        last_right_wheel_pos_ = current_right_pos;

        // Publish Odometry & TF (Helper struct for quaternion math)
        double half_yaw = odom_theta_ * 0.5;
        double q_z = std::sin(half_yaw);
        double q_w = std::cos(half_yaw);

        if (odom_pub_->get_subscription_count() > 0) {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = get_node()->get_clock()->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = odom_x_;
            odom_msg.pose.pose.position.y = odom_y_;
            odom_msg.pose.pose.orientation.z = q_z;
            odom_msg.pose.pose.orientation.w = q_w;
            odom_msg.twist.twist.linear.x = linear_distance / period.seconds();
            odom_msg.twist.twist.angular.z = angular_distance / period.seconds();
            odom_pub_->publish(odom_msg);
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = get_node()->get_clock()->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = odom_x_;
        tf_msg.transform.translation.y = odom_y_;
        tf_msg.transform.rotation.z = q_z;
        tf_msg.transform.rotation.w = q_w;
        tf_broadcaster_->sendTransform(tf_msg);


        // --- Third, state logic ---
        auto twist_msg = twist_cmd_buffer_.readFromRT();
        auto joy_msg = joy_cmd_buffer_.readFromRT();

        if (joy_msg && joy_msg->buttons.size() >= 4) {
            if (joy_msg->buttons[STATE_IDLE_B] == 1) {
                current_state_ = TibbleState::IDLE;
            } else if (joy_msg->buttons[STATE_TRAVEL_B] == 1) {
                current_state_ = TibbleState::TRAVEL;
            } else if (joy_msg->buttons[STATE_EXCAVATE_B] == 1) {
                current_state_ = TibbleState::EXCAVATE;
            } else if (joy_msg->buttons[STATE_DUMP_B] == 1) {
                current_state_ = TibbleState::DUMP;
            }
        }

        // Timer management for sequential steps in state transitions
        if (current_state_ != previous_state_) {
            state_timer_ = 0.0;
            previous_state_ = current_state_;
        } else {
            state_timer_ += period.seconds();
        }


        // --- Fourth, kinematics and output commands ---
        double target_v = 0.0;
        double target_w = 0.0;
        
        if (twist_msg && twist_msg) {
            target_v = twist_msg->linear.x;
            target_w = twist_msg->angular.z;
        }

        // Initialize safe defaults
        double cmd_left_wheel = 0.0;
        double cmd_right_wheel = 0.0;
        double cmd_la_pos = LA_REST_POS;
        double cmd_excav_vel = 0.0;
        double cmd_vibe = 0.0;     // 0.0 = off, 1.0 = on
        double cmd_latch = 1.0;    // 1.0 = latched, 0.0 = unlatched

        double speed_multiplier = 1.0;

        // --- THE STATE MACHINE ---
        switch (current_state_) {
            
            case TibbleState::IDLE:
                // Safe defaults are already set above
                break;

            case TibbleState::TRAVEL:
                // Only change is drivetrain is on, logic handled outside switch 
                break;

            case TibbleState::EXCAVATE:
                speed_multiplier = 0.5; // Slow down drivetrain while excavating
                
                // Sequence 1: Start vibe immediately
                cmd_vibe = 1.0;

                // Sequence 2: Set LA's to EXCAV
                cmd_la_pos = LA_EXCAV_POS;

                // Sequence 3: Only start paddles if LA is fully retracted (within error)
                if (std::abs(current_la_pos - LA_EXCAV_POS) < 0.01) {
                    cmd_excav_vel = paddle_speed_;
                }
                break;

            case TibbleState::DUMP:
                speed_multiplier = 0.3; // Slow down drivetrain while dumping
                
                // Sequence 1: Unlatch immediately
                cmd_latch = 0.0;

                // Sequence 2: Extend LAs after latch has time to open
                if (state_timer_ > DUMP_LA_DELAY) {
                    cmd_la_pos = LA_DUMP_POS;
                }

                // Sequence 3: Turn on vibe motor to shake out regolith once fully tipped
                if (state_timer_ > DUMP_VIBE_DELAY) {
                    cmd_vibe = 1.0;
                }
                break;
        }

        // Differential Drive Math (applied for TRAVEL, EXCAVATE, and DUMP)
        if (current_state_ != TibbleState::IDLE) {
            target_v *= speed_multiplier;   // apply speed multiplier if necessary
            target_w *= speed_multiplier;

            cmd_left_wheel = (target_v - target_w * wheel_separation_ / 2.0) / wheel_radius_;
            cmd_right_wheel = (target_v + target_w * wheel_separation_ / 2.0) / wheel_radius_;
        }


        // --- Fourth, write commands ---
        
        // Order is declaration order in command_interface_configuration()
        (void)command_interfaces_[0].set_value(cmd_left_wheel);
        (void)command_interfaces_[1].set_value(cmd_right_wheel);
        (void)command_interfaces_[2].set_value(cmd_la_pos);
        (void)command_interfaces_[3].set_value(cmd_excav_vel);
        (void)command_interfaces_[4].set_value(cmd_vibe);
        (void)command_interfaces_[5].set_value(cmd_latch);

        return controller_interface::return_type::OK;
    }

} // namespace tibble_controller


PLUGINLIB_EXPORT_CLASS(tibble_controller::TibbleController,
                       controller_interface::ControllerInterface)
