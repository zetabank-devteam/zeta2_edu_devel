#include "zeta_if.hpp"


int main(int argc, char** argv){

    int rate_b = 100;

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a shared pointer to your node
    auto node = std::make_shared<Zeta_IF>("zeta_if_node");

    // Initialize any components of your node if necessary
    node->Init();
    node->InitSerial();

    // Set the rate at which to run the loop
    rclcpp::Rate rate(rate_b); 
    
    // Spin the node
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Cleanup and shutdown
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Exit Zeta Interface Process");

    return 0;
}