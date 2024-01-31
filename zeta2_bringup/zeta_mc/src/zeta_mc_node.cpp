#include "zeta_mc.hpp"

int main (int argc, char** argv){
 
    int rate_b = 100;

    // Initialize ROS2
    rclcpp::init(argc, argv);
   
    // Create a shared pointer to your node
    auto node = std::make_shared<Zeta_MC>("zeta_mc_node");

    // Initialize any components of your node if necessary
    node->Init();
	node->InitSerial();
    
    // Set the rate at which to run the loop
    rclcpp::Rate rate(rate_b); 
   

#ifdef DEBUG_BASIC
    RCLCPP_INFO(node->get_logger(), "End process!!!");
#endif

    // Spin the node
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Cleanup and shutdown
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Exit Zeta Motor Control Process!!!");

    return 0;
}