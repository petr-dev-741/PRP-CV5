
//#include <oneapi/tbb/partitioner.h> // throws error when building
#include <memory>
#include "nodes/maze_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto ImuClass = std::make_shared<nodes::MazeNode>();

    executor->add_node(ImuClass);

    std::cout << "Hello World!" << std::endl;

    executor->spin();

    rclcpp::shutdown();
    return 0;
}
