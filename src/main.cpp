
//#include <oneapi/tbb/partitioner.h> // throws error when building
#include <memory>
#include "nodes/io_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto IOnodeClass = std::make_shared<nodes::IoNode>();

    executor->add_node(IOnodeClass);

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
