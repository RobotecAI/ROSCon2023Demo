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
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
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
#include "taskConstructor.h"
#include "ur_moveit_demo_msg/action/mtc.hpp"
#include "utils.h"
#include "vision.h"

constexpr char BoxNamePrefix[] = "Box";
constexpr char PalletNamePrefix[] = "EuroPallet";
constexpr char PickedBoxName[] = "PickedBox";

const std::vector<Eigen::Vector3f> Pattern{
    { -1.f, -0.5f, 1.f }, { 0.f, -0.5f, 1.f }, { 1.f, -0.5f, 1.f }, { -1.f, 0.5, 1.f }, { 0.f, 0.5f, 1.f }, { 1.f, 0.5f, 1.f },
    { -1.f, -0.5f, 2.f }, { 0.f, -0.5f, 2.f }, { 1.f, -0.5f, 2.f }, { -1.f, 0.5, 2.f }, { 0.f, 0.5f, 2.f }, { 1.f, 0.5f, 2.f },
};

class MTCActionServer
{
public:
    using MTCAction = ur_moveit_demo_msg::action::Mtc;
    using GoalHandleMTC = rclcpp_action::ServerGoalHandle<MTCAction>;

    MTCActionServer(rclcpp::Node::SharedPtr node, std::string ns)
        : node_(node)
        , ns_(ns)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<MTCAction>(
            node->get_node_base_interface(),
            node->get_node_clock_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            "MTC",
            std::bind(&MTCActionServer::handle_goal, this, _1, _2),
            std::bind(&MTCActionServer::handle_cancel, this, _1),
            std::bind(&MTCActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<MTCAction>::SharedPtr action_server_;
    rclcpp::Node::SharedPtr node_;
    std::string ns_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const MTCAction::Goal> goal)
    {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMTC> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMTC> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{ std::bind(&MTCActionServer::execute, this, _1), goal_handle }.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMTC> goal_handle)
    {
        auto result = std::make_shared<MTCAction::Result>();

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/" + ns_);
        planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

        auto mtc_task_node = std::make_shared<TaskConstructor::MTCController>(node_, ns_);
        auto gripperController = std::make_shared<Gripper::GripperController>(node_, "/" + ns_ + "/gripper_server");
        auto visionSystem = std::make_shared<Camera::GroundTruthCamera>(
            node_, "/" + ns_ + "/camera_pickup/detections3D", "/" + ns_ + "/camera_drop/detections3D", ns_);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MTCAction::Feedback>();
        for (int i = 0; i < goal->num_of_boxes; i++)
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                return;
            }
            auto const& address = Pattern[i];
            std::optional<geometry_msgs::msg::Pose> myClosestBox;
            std::vector<geometry_msgs::msg::Pose> allBoxesOnPallet;

            myClosestBox = visionSystem->getClosestBox();
            allBoxesOnPallet = visionSystem->getAllBoxesOnPallet();

            auto palletPose = visionSystem->getObjectPose("EuroPallet");
            if (!palletPose) {
                goal_handle->abort(result);
                return;
            }

            if (!myClosestBox)
            {
                RCLCPP_ERROR(node_->get_logger(), "No box found");
                goal_handle->abort(result);
                return;
            }

            auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");

            mtc::Task taskGrab = mtc_task_node->createTaskGrab(*myClosestBox, PickedBoxName, ns_);
            if (!mtc_task_node->doTask(taskGrab))
            {
                goal_handle->abort(result);
                return;
            }
            gripperController->Grip();
            auto taskDrop = mtc_task_node->createTaskDrop(address, PickedBoxName, *palletPose, *myClosestBox, allBoxesOnPallet, ns_);
            if (!mtc_task_node->doTask(taskDrop))
            {
                gripperController->Release();
                goal_handle->abort(result);
                return;
            }
            gripperController->Release();
            feedback->current_box = i;
            goal_handle->publish_feedback(feedback);
        }

        mtc::Task taskPark = mtc_task_node->createTaskPark(ns_);
        mtc_task_node->doTask(taskPark);

        // Check if goal is done
        if (rclcpp::ok())
        {
            result->success = true;
            goal_handle->succeed(result);
        }
    }
};

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
        std::make_shared<rclcpp::Node>("mtc_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread(
        [&executor]()
        {
            executor.spin();
        });

    auto parameter = node->get_parameter("ns");
    auto ns = parameter.as_string();

    auto mtc_action_server = std::make_shared<MTCActionServer>(node, ns);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
