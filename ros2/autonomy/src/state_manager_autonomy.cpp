#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <interfaces/srv/set_tibble_state.hpp>

namespace tibble_autonomy
{

class SetTibbleStateNode : public BT::SyncActionNode
{
public:
    SetTibbleStateNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        // Extract the ROS 2 node from the BT blackboard so we can create a client
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = node_->create_client<interfaces::srv::SetTibbleState>("/tibble_controller/set_state");
    }

    // This defines the input ports available in the XML file
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("requested_state", "0=IDLE, 1=TRAVEL, 2=EXCAVATE, 3=DUMP")
        };
    }

    // The logic executed when the tree reaches this node
    BT::NodeStatus tick() override
    {
        int target_state;
        if (!getInput<int>("requested_state", target_state)) {
            RCLCPP_ERROR(node_->get_logger(), "BT Node missing required input: [requested_state]");
            return BT::NodeStatus::FAILURE;
        }

        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "State service not available. BT halting.");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<interfaces::srv::SetTibbleState::Request>();
        request->requested_state = target_state;

        // Call the service synchronously (we block the BT thread until we get a response)
        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Autonomy successfully commanded state: %d", target_state);
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_WARN(node_->get_logger(), "Autonomy state transition failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Service call timed out.");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<interfaces::srv::SetTibbleState>::SharedPtr client_;
};

} // namespace tibble_autonomy

// Register the node as a plugin
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<tibble_autonomy::SetTibbleStateNode>("SetTibbleState");
}