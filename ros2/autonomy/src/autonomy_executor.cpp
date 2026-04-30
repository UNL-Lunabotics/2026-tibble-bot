#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create the ROS 2 Node
    auto node = std::make_shared<rclcpp::Node>("autonomy_executor");

    // Declare a parameter for the XML file name so we can change missions via launch files
    node->declare_parameter("tree_xml_file", "main_tree.xml");
    std::string tree_xml_file = node->get_parameter("tree_xml_file").as_string();

    // Dynamically find the path to the XML file in the ROS 2 install/share directory
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("autonomy");
    std::string xml_path = package_share_dir + "/behavior_trees/" + tree_xml_file;

    RCLCPP_INFO(node->get_logger(), "Loading Behavior Tree from: %s", xml_path.c_str());

    // 1. Create the BT Factory
    BT::BehaviorTreeFactory factory;

    // 2. Setup the Blackboard
    // The blackboard is a shared memory space. We MUST put our ROS 2 node pointer 
    // in here so that our custom plugins can find it and use it to make Service/Action clients.
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);

    // 3. Register Plugins
    // This loads the compiled .so file of your custom state manager node
    try {
        factory.registerFromPlugin("libstate_manager_autonomy_plugin.so");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load custom BT plugin: %s", e.what());
        return 1;
    }

    // 4. Create the Tree
    BT::Tree tree = factory.createTreeFromFile(xml_path, blackboard);

    // 5. Execution Loop (Tick the tree at ~10Hz)
    rclcpp::Rate rate(10);
    bool finish = false;
    
    RCLCPP_INFO(node->get_logger(), "Mission Started. Ticking tree...");
    
    while (rclcpp::ok() && !finish) {
        // Spin the ROS node slightly to process any incoming service replies or callbacks
        rclcpp::spin_some(node);
        
        // Tick the root node of the tree
        BT::NodeStatus status = tree.tickExactlyOnce();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Mission Completed Successfully!");
            finish = true;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "Mission Failed.");
            finish = true;
        }
        
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}