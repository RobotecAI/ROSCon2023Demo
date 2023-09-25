
#include "otto_deliberation/robot_status.h"
#include "otto_deliberation/tasks.h"
#include <chrono>
#include <functional>
#include <lane_provider_msgs/srv/list_tracks.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <otto_deliberation/otto_autonomy.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>

class OttoDeliberation
{
public:
    OttoDeliberation(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lock_node)
        : m_node(node)
        , m_autonomy(node, lock_node)
    {
        RCLCPP_INFO(m_node->get_logger(), "Otto deliberation node starting");
        m_namespace = m_node->get_namespace();
        m_timer = m_node->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OttoDeliberation::TimerCallback, this));
    }

    bool Initialize()
    {
        std::string assigned_lane_name = m_node->declare_parameter<std::string>("assigned_lane", "");
        m_node->get_parameter<std::string>("assigned_lane", assigned_lane_name);

        if (assigned_lane_name.empty())
        {
            RCLCPP_ERROR(m_node->get_logger(), "No assigned lane, terminating");
            return false;
        }

        std::string lane_track_service = m_node->declare_parameter<std::string>("lane_track_service", "/get_lanes_and_paths");
        m_node->get_parameter<std::string>("lane_track_service", lane_track_service);
        auto laneTracksClient = m_node->create_client<lane_provider_msgs::srv::ListTracks>(lane_track_service);
        if (!laneTracksClient->wait_for_service(std::chrono::seconds(2)))
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
        auto lane_paths = lane_track_response_future.get()->lane_paths;
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

        Tasks tasks;
        nav_msgs::msg::Path empty;
        tasks.push_back(MakeTask("idle", empty));
        for (size_t i = 0; i < assigned_lane.path_names.size(); ++i)
        { // Construct Tasks
            tasks.push_back(MakeTask(assigned_lane.path_names[i], assigned_lane.lane_paths[i]));
        }
        m_autonomy.SetTasks(tasks);
        m_autonomy.SetLane(assigned_lane_name);

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
    }

    Task MakeTask(std::string path_name, NavPath path)
    {
        Task t;
        t.m_taskKey = path_name;
        t.m_path = path;
        t.m_goalTaskStatus = TaskUtils::GetTaskStatus(path_name);
        t.m_requiredCargoStatus = TaskUtils::GetCargoStatus(path_name);
        t.m_reverse = TaskUtils::GetReverse(path_name);
        t.m_requiresLock = true;

        if (path_name.find("GoToWrapping") != std::string::npos) {
            t.m_wait = true;
            t.m_waitTime = std::chrono::seconds(5);
        }
        else {
            t.m_wait = false;
        }

        return t;
    }

    rclcpp::Node::SharedPtr m_node;
    std::atomic_bool m_cargoLoaded{ false };
    rclcpp::TimerBase::SharedPtr m_timer;
    std::string m_namespace;
    OttoAutonomy m_autonomy;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_cargoStatusSubscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions otto_node_options;
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
