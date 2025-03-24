
//#include <oneapi/tbb/partitioner.h> // throws error when building
#include <memory>
#include "nodes/io_node.hpp"
#include "nodes/LineSensorNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto IOnodeClass = std::make_shared<nodes::IoNode>();
    //auto IOnodeLineSensor = std::make_shared<nodes::LineSensorNode>();

    executor->add_node(IOnodeClass);
    // executor->add_node(IOnodeLineSensor);

    std::cout << "Hello World!" << std::endl;

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
