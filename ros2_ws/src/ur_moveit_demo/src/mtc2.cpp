#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/task_constructor/task.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>

namespace mtc = moveit::task_constructor;

mtc::Task createScene(tf2_ros::Buffer& tf_buffer_) {
    const Eigen::Vector3d TableDimension{ 0.950, 0.950, 0.411 };
    const Eigen::Vector3d ConveyorDimensions{ 2.0, 1., 0.15 };

    return mtc::Task();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto const node =
        std::make_shared<rclcpp::Node>("mtc_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread(
        [&executor]()
        {
            executor.spin();
        });


    auto tf_buffer_ = tf2_ros::Buffer(node->get_clock());
    auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    std::this_thread::sleep_for(std::chrono::seconds(1));



    rclcpp::shutdown();
    spinner.join();
}