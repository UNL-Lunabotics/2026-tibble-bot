#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <interfaces/srv/set_tibble_state.hpp>
#include <vector>

class StateManagerNode : public rclcpp::Node
{
public:
    StateManagerNode() : Node("state_manager_node")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&StateManagerNode::joy_callback, this, std::placeholders::_1));
        
        client_ = this->create_client<interfaces::srv::SetTibbleState>("/tibble_controller/set_state");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (previous_buttons_.empty()) {
            previous_buttons_ = msg->buttons;
            return;
        }

        // Helper lambda for edge detection
        auto button_just_pressed = [&](int btn_idx) {
            if (msg->buttons.size() > (size_t)btn_idx && previous_buttons_.size() > (size_t)btn_idx) {
                return msg->buttons[btn_idx] == 1 && previous_buttons_[btn_idx] == 0;
            }
            return false;
        };

        int target_state = -1;
        if (button_just_pressed(0)) { target_state = interfaces::srv::SetTibbleState::Request::IDLE; }
        else if (button_just_pressed(1)) { target_state = interfaces::srv::SetTibbleState::Request::TRAVEL; }
        else if (button_just_pressed(2)) { target_state = interfaces::srv::SetTibbleState::Request::EXCAVATE; }
        else if (button_just_pressed(3)) { target_state = interfaces::srv::SetTibbleState::Request::DUMP; }

        if (target_state != -1) {
            send_state_request(target_state);
        }

        previous_buttons_ = msg->buttons;
    }

    void send_state_request(int target_state)
    {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "State service not available, skipping request.");
            return;
        }
        auto request = std::make_shared<interfaces::srv::SetTibbleState::Request>();
        request->requested_state = target_state;
        client_->async_send_request(request); // Non-blocking async call
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<interfaces::srv::SetTibbleState>::SharedPtr client_;
    std::vector<int> previous_buttons_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
