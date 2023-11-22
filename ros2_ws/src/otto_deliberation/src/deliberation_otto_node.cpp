#include "otto_deliberation/otto_autonomy.h"
#include "otto_deliberation/robot_status.h"
#include "otto_deliberation/tasks.h"
#include <lane_provider_msgs/srv/list_tracks.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <functional>
#include <string>
#include <thread>

//! Node that handles the deliberation of the Otto robot
//! It allows to create a list of tasks that the robot will execute in sequence
//! It also allows to lock the robot to a specific path communicating with orchestrator
//! It accepts following parameters:
//! - assigned_lane: the name of the lane assigned to the robot
//! - tasks: a list of tasks to be executed by the robot
//! - lifter_tasks: a list of tasks that will be executed with lifter lifted
//! - dummy_tasks: a list of tasks that will be executed by the dummy (no robot movement)
//! - blind_tasks: a list of tasks that will be executed with the robot blind (no nav2 action used)
//! - blind_task_reverse: a list of tasks that will be executed with the robot blind and in reverse (no nav2 action used)
//! - cargo_unload_tasks: a list of tasks that will be executed with the robot unloading cargo
//! - cargo_load_tasks: a list of tasks that will be executed with the robot loading cargo
//! - task_acquire_lock : a list of tasks that will be executed with the robot locked to a path
//! - task_release_lock : a list of task that will release lock on the end of the task
//! - pre_task_delay: a list of doubles that represent the delay in seconds to wait before executing the next task
//! - post_task_delay: a list of doubles that represent the delay in seconds to wait after executing the next task
//! - loop: if true, the robot will loop the list of tasks
//! - lock_service: the name of the lock service
//! It publishes following topics:
//! - deliberation_description: a string that describes the current task
//! - deliberation_state: a string that describes the current task name
//! - lifter: set state of the lifter
//! It uses following services:
//! - /lock_service : a global service that locks the robot to a specific path
//! - /get_lanes_and_paths : a global service that returns the list of available tracks
//! It uses following actions:
//! - navigate_through_poses: a nav2 action that allows to navigate through a list of poses
//! - blind_follow_path: an action that allows to follow a path without using nav2

class OttoDeliberation
{
public:
    OttoDeliberation(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lockNode)
        : m_node(node)
        , m_autonomy(node, lockNode)
    {
        RCLCPP_INFO(m_node->get_logger(), "Otto deliberation node starting");
        m_namespace = m_node->get_namespace();
        m_timer = m_node->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OttoDeliberation::TimerCallback, this));
        m_deliberationStatusDescriptionPublisher = m_node->create_publisher<std_msgs::msg::String>("deliberation_description", 10);
        m_deliberationStatePublisher = m_node->create_publisher<std_msgs::msg::String>("deliberation_state", 10);
    }

    bool Initialize()
    {
        const std::string assigned_lane_name = m_node->declare_parameter<std::string>("assigned_lane", "");
        if (assigned_lane_name.empty())
        {
            RCLCPP_ERROR(m_node->get_logger(), "No assigned lane, terminating");
            return false;
        }

        RobotTasks tasks;
        tasks.m_loop = m_node->declare_parameter<bool>("loop", false);
        tasks.m_tasks = PrepareNamedTaskList("tasks");
        tasks.m_dummyTasks = PrepareNamedTaskSet("dummy_tasks");
        tasks.m_lifterTasks = PrepareNamedTaskSet("lifter_tasks");
        tasks.m_blindTasks = PrepareNamedTaskSet("blind_tasks");
        tasks.m_blindTasksReverse = PrepareNamedTaskSet("blind_tasks_reverse");
        tasks.m_cargoLoadTasks = PrepareNamedTaskSet("cargo_load_tasks");
        tasks.m_cargoUnLoadTasks = PrepareNamedTaskSet("cargo_unload_tasks");
        tasks.m_acquireLock = PrepareNamedTaskSet("task_acquire_lock");
        tasks.m_releaseLock = PrepareNamedTaskSet("task_release_lock");
        tasks.m_blindHighSpeed = PrepareNamedTaskSet("blind_tasks_high_speed");

        if (!PrepareTaskDelays(tasks) || !PrepareTaskPaths(tasks, assigned_lane_name) || !tasks.ValidateTasks())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Task list is invalid, terminating");
            return false;
        }

        m_autonomy.SetLane(assigned_lane_name);
        m_autonomy.SetTasks(tasks);

        m_cargoStatusSubscriber = m_node->create_subscription<std_msgs::msg::Bool>(
            m_namespace + "/cargo_status",
            1,
            [&](std_msgs::msg::Bool b)
            {
                m_cargoLoaded = b.data;
            });

        return true;
    }

private:
    void TimerCallback()
    { // Get tasks, updates etc. here
        RobotStatus robotStatus = m_autonomy.GetCurrentStatus();
        bool robotLoaded = robotStatus.m_cargoStatus == RobotCargoStatus::CARGO_LOADED;
        if (m_cargoLoaded != robotLoaded)
        { // Cargo status changed - notify
            m_autonomy.NotifyCargoChanged(m_cargoLoaded);
        }

        m_autonomy.Update();

        std_msgs::msg::String msgDescription;
        msgDescription.data = m_autonomy.GetCurrentOperationDescription();
        m_deliberationStatusDescriptionPublisher->publish(msgDescription);

        std_msgs::msg::String msgTaskName;
        msgTaskName.data = m_autonomy.GetCurrentTaskName();
        m_deliberationStatePublisher->publish(msgTaskName);
    }

    std::vector<std::string> PrepareNamedTaskList(const std::string& taskListName)
    {
        // we need to filter out empty elements from the task lists to overcome
        // https://github.com/ros2/rclcpp/issues/1955
        auto taskList = m_node->declare_parameter<std::vector<std::string>>(taskListName, { "" });
        taskList.erase(
            std::remove_if(
                taskList.begin(),
                taskList.end(),
                [](const auto& taskName)
                {
                    return taskName.empty();
                }),
            taskList.end());
        return taskList;
    }

    RobotTaskSet PrepareNamedTaskSet(const std::string& taskSetName)
    {
        const auto taskList = PrepareNamedTaskList(taskSetName);
        return { taskList.begin(), taskList.end() };
    }

    bool PrepareTaskDelays(RobotTasks& tasks)
    {
        const auto preTaskDelays = m_node->declare_parameter<std::vector<double>>("pre_task_delays", { 0.0 });
        const auto postTaskDelays = m_node->declare_parameter<std::vector<double>>("post_task_delays", { 0.0 });

        if (preTaskDelays.size() != tasks.m_tasks.size())
        {
            RCLCPP_ERROR(m_node->get_logger(), "pre_task_delays size %zu != tasks size %zu", preTaskDelays.size(), tasks.m_tasks.size());
            return false;
        }
        if (postTaskDelays.size() != tasks.m_tasks.size())
        {
            RCLCPP_ERROR(m_node->get_logger(), "postTaskDelays size %zu != tasks size %zu", postTaskDelays.size(), tasks.m_tasks.size());
            return false;
        }

        for (size_t i = 0; i < tasks.m_tasks.size(); i++)
        {
            const auto& taskName = tasks.m_tasks[i];
            const double preTaskDelay = preTaskDelays[i];
            const double postTaskDelay = postTaskDelays[i];
            if (preTaskDelay > 0.0)
            {
                tasks.m_preTaskDelay[taskName] = preTaskDelay;
            }
            if (postTaskDelay > 0.0)
            {
                tasks.m_preTaskDelay[taskName] = postTaskDelay;
            }
        }

        return true;
    }

    bool PrepareTaskPaths(RobotTasks& tasks, const std::string& assigned_lane_name)
    {
        const std::string lane_track_service = m_node->declare_parameter<std::string>("lane_track_service", "/get_lanes_and_paths");
        auto laneTracksClient = m_node->create_client<lane_provider_msgs::srv::ListTracks>(lane_track_service);
        if (!laneTracksClient->wait_for_service(std::chrono::seconds(30)))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Lane track service named %s not available", lane_track_service.c_str());
            return false;
        }

        auto lane_request = std::make_shared<lane_provider_msgs::srv::ListTracks::Request>();
        lane_request->lane_name = assigned_lane_name;
        auto lane_track_response_future = laneTracksClient->async_send_request(lane_request);
        if (rclcpp::spin_until_future_complete(m_node, lane_track_response_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to call service %s", lane_track_service.c_str());
            return false;
        }
        const auto lane_paths = lane_track_response_future.get()->lane_paths;
        if (lane_paths.empty())
        {
            RCLCPP_ERROR(m_node->get_logger(), "No paths returned for lane %s", assigned_lane_name.c_str());
            return false;
        }
        auto assigned_lane = lane_paths[0];

        if (assigned_lane.path_names.empty() || assigned_lane.path_names.size() != assigned_lane.lane_paths.size())
        {
            RCLCPP_ERROR(m_node->get_logger(), "Empty or incorrectly specified paths specified for lane %s", assigned_lane_name.c_str());
            return false;
        }

        for (size_t i = 0; i < assigned_lane.path_names.size(); ++i)
        {
            if (tasks.m_taskPaths.find(assigned_lane.path_names[i]) != tasks.m_taskPaths.end())
            {
                RCLCPP_ERROR(
                    m_node->get_logger(),
                    "Duplicate path name %s specified for lane %s - assigning new Nav path",
                    assigned_lane.path_names[i].c_str(),
                    assigned_lane_name.c_str());
            }
            tasks.m_taskPaths[assigned_lane.path_names[i]] = std::make_shared<NavPath>(std::move(assigned_lane.lane_paths[i]));
        }

        return true;
    }

    rclcpp::Node::SharedPtr m_node;
    std::atomic_bool m_cargoLoaded{ false };
    rclcpp::TimerBase::SharedPtr m_timer;
    std::string m_namespace;
    OttoAutonomy m_autonomy;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_cargoStatusSubscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_deliberationStatusDescriptionPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_deliberationStatePublisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto otto_node = std::make_shared<rclcpp::Node>("otto_deliberation_node");
    auto otto_lock_node = std::make_shared<rclcpp::Node>("otto_lock_node");
    OttoDeliberation otto_deliberation(otto_node, otto_lock_node);
    if (!otto_deliberation.Initialize())
    {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(otto_lock_node);

    std::thread spinner(
        [&]()
        {
            executor.spin();
        });
    rclcpp::spin(otto_node);
    spinner.join();

    rclcpp::shutdown();
    RCLCPP_INFO(otto_node->get_logger(), "Otto terminating");
    return 0;
}
