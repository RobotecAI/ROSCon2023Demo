#include <chrono>
#include <control_msgs/action/detail/gripper_command__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vision_msgs/msg/detection3_d_array.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "gripper.h"
#include "vision.h"
#include "utils.h"
#include "taskConstructor.h"

constexpr float BoxHeight = 0.3f;
const Eigen::Vector3d TableDimension{ 0.950, 0.950, 0.411 };
const Eigen::Vector3d ConveyorDimensions{ 2.0, 1., 0.15 };
const Eigen::Vector3d PickupLocation{ 0.890, 0, 0.049 };
const Eigen::Vector3d BoxDimension{ 0.2, 0.2, 0.2 };
const Eigen::Vector3d PalletDimensions{ 1.2, 0.769, 0.111 };

constexpr char BoxNamePrefix[] = "Box";
constexpr char PalletNamePrefix[] = "EuroPallet";
constexpr char PickedBoxName[] = "PickedBox";


int main(int argc, char** argv)
{
    std::mutex locationsMutex;
    std::optional<geometry_msgs::msg::Pose> closestBox;
    std::map<std::string, geometry_msgs::msg::Pose> objectPoses;
    std::vector<geometry_msgs::msg::Pose> allBoxesOnPallet;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    rclcpp::executors::MultiThreadedExecutor executor_mtc;

    auto const node =
        std::make_shared<rclcpp::Node>("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    const std::vector<Eigen::Vector3f> Pattern{
        { -1.f, -0.5f, 1.f }, { 0.f, -0.5f, 1.f }, { 1.f, -0.5f, 1.f }, { -1.f, 0.5, 1.f }, { 0.f, 0.5f, 1.f }, { 1.f, 0.5f, 1.f },
        { -1.f, -0.5f, 2.f }, { 0.f, -0.5f, 2.f }, { 1.f, -0.5f, 2.f }, { -1.f, 0.5, 2.f }, { 0.f, 0.5f, 2.f }, { 1.f, 0.5f, 2.f },
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread(
        [&executor]()
        {
            executor.spin();
        });

    auto parameter = node->get_parameter("ns");
    auto ns = parameter.as_string();

    auto mtc_task_node = std::make_shared<TaskConstructor::MTCController>(node, ns);

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface::Options optionsmgi(ns + "/ur_manipulator", "robot_description", "/" + ns);
    auto move_group_interface = MoveGroupInterface(node, optionsmgi);

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, ns + "/base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
    };
    moveit_visual_tools.loadRemoteControl();
    moveit_visual_tools.trigger();

    auto gripperController = Gripper::GripperController(node, "/" + ns + "/gripper_server");
    // auto tf_buffer_ = tf2_ros::Buffer(node->get_clock());
    // auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/" + ns);

    Camera::GroundTruthCamera visionSystem(node, "/" + ns + "/camera_pickup/detections3D", "/" + ns + "/camera_drop/detections3D", ns);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // find pallet pose
    auto palletPose = visionSystem.getObjectPose("EuroPallet");

    // visualize pallet and boxes
    moveit_visual_tools.publishCuboid(
        palletPose, PalletDimensions.x(), PalletDimensions.y(), PalletDimensions.z(), rviz_visual_tools::BROWN);

    for (auto& p : Pattern)
    {
        auto targetPose = Utils::getBoxTargetPose(p, palletPose, BoxDimension);
        moveit_visual_tools.publishCuboid(targetPose, BoxDimension.x(), BoxDimension.y(), BoxDimension.z(), rviz_visual_tools::YELLOW);

        moveit_visual_tools.publishArrow(targetPose, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    }
    moveit_visual_tools.trigger();

    planning_scene_interface.applyCollisionObject(
        Utils::CreateBoxCollision("pallet", PalletDimensions, Utils::fromMsgPosition(palletPose.position), Utils::fromMsgQuaternion(palletPose.orientation)));

    moveit_visual_tools.prompt("  ");

    for (auto const& address : Pattern)
    {
        std::optional<geometry_msgs::msg::Pose> myClosestBox;

        myClosestBox = visionSystem.getClosestBox();
        allBoxesOnPallet = visionSystem.getAllBoxesOnPallet();

        if (!myClosestBox)
        {
            RCLCPP_ERROR(node->get_logger(), "No box found");
            return 1;
        }

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");

        mtc::Task taskGrab = mtc_task_node->createTaskGrab(*myClosestBox, PickedBoxName, ns);
        mtc_task_node->doTask(taskGrab);
        gripperController.Grip();
        auto taskDrop = mtc_task_node->createTaskDrop(address, PickedBoxName, palletPose, *myClosestBox, allBoxesOnPallet, ns);
        mtc_task_node->doTask(taskDrop);
        gripperController.Release();
    }

    mtc::Task taskPark = mtc_task_node->createTaskPark(ns);
    mtc_task_node->doTask(taskPark);
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
