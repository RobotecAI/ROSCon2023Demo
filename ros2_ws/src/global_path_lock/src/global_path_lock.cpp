#include <cassert>
#include <functional>
#include <lane_provider_msgs/srv/list_tracks.hpp>
#include <lock_service_msgs/srv/detail/lock__struct.hpp>
#include <lock_service_msgs/srv/lock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

class LockService
{
    using PathLocks = std::map<std::string, bool>;

public:
    LockService(rclcpp::Node::SharedPtr node)
        : m_node(node)
    {
        m_service = node->create_service<lock_service_msgs::srv::Lock>(
            "/lock_service", std::bind(&LockService::LockSrv, this, std::placeholders::_1, std::placeholders::_2));
    };

    void LockSrv(
        const std::shared_ptr<lock_service_msgs::srv::Lock::Request> request,
        std::shared_ptr<lock_service_msgs::srv::Lock::Response> response)
    {
        if (request->lock_status == true)
        {
            bool isLocked = IsPathLocked(request->lane_name, request->path_name);
            if (!isLocked)
            {
                SetPathLock(request->lane_name, request->path_name, true);
            }
            response->result = !isLocked;
        }
        else
        {
            SetPathLock(request->lane_name, request->path_name, false);
            response->result = true;
        }
    };

    template<typename Container>
    Container filterEmptyElements(const Container& container)
    {
        Container filteredContainer;
        using T = typename Container::value_type;
        std::copy_if(
            container.begin(),
            container.end(),
            std::back_inserter(filteredContainer),
            [](const T element)
            {
                return element != T();
            });

        return filteredContainer;
    }

    bool Initialize()
    {
        std::string lane_track_service = m_node->declare_parameter<std::string>("lane_track_service", "/get_lanes_and_paths");
        m_node->get_parameter<std::string>("lane_track_service", lane_track_service);
        auto laneTracksClient = m_node->create_client<lane_provider_msgs::srv::ListTracks>(lane_track_service);
        if (!laneTracksClient->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(m_node->get_logger(), "Lane track service named %s not available", lane_track_service.c_str());
            return false;
        }

        auto lane_request = std::make_shared<lane_provider_msgs::srv::ListTracks::Request>();
        auto lane_track_response_future = laneTracksClient->async_send_request(lane_request);
        if (rclcpp::spin_until_future_complete(m_node, lane_track_response_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to call service %s", lane_track_service.c_str());
            return false;
        }

        auto allLanes = lane_track_response_future.get();
        for (auto lane : allLanes->lane_paths)
        {
            PathLocks locks;
            for (auto path : lane.path_names)
            {
                locks.insert({ path, false });
            }
            m_lanesLocks.insert({ lane.lane_name, locks });
        }

        auto globalPaths = m_node->declare_parameter<std::vector<std::string>>("global_paths", { "" });
        auto dependenciesFirst = m_node->declare_parameter<std::vector<std::string>>("dependencies_first", { "" });
        auto dependenciesSecond = m_node->declare_parameter<std::vector<std::string>>("dependencies_second", { "" });

        // we need to filter out empty elements from the task lists to overcome
        // https://github.com/ros2/rclcpp/issues/1955
        const auto globalPathsFiltered = filterEmptyElements(globalPaths);
        const auto dependenciesFirstFiltered = filterEmptyElements(dependenciesFirst);
        const auto dependenciesSecondFiltered = filterEmptyElements(dependenciesSecond);

        if (dependenciesFirstFiltered.size() != dependenciesSecondFiltered.size())
        {
            RCLCPP_ERROR(
                m_node->get_logger(),
                "dependencies_first size %zu != dependencies_second size %zu",
                dependenciesFirstFiltered.size(),
                dependenciesSecondFiltered.size());
            return false;
        }
        for (auto path : globalPathsFiltered)
        {
            m_globalPaths.insert(path);
        }
        for (int i = 0; i < dependenciesFirstFiltered.size(); i++)
        {
            m_dependencies.insert({ dependenciesFirstFiltered[i], dependenciesSecondFiltered[i] });
        }

        return true;
    }

private:
    rclcpp::Node::SharedPtr m_node;

    std::map<std::string, PathLocks> m_lanesLocks;

    std::set<std::string> m_globalPaths;
    std::map<std::string, std::string> m_dependencies;

    rclcpp::Service<lock_service_msgs::srv::Lock>::SharedPtr m_service;

    std::optional<std::string> PathDependency(const std::string& path_name)
    {
        if (m_dependencies.find(path_name) == m_dependencies.end())
        {
            return std::optional<std::string>();
        }
        return std::optional<std::string>(m_dependencies.find(path_name)->second);
    };

    bool IsPathGlobal(const std::string& path_name)
    {
        return m_globalPaths.count(path_name) == 1;
    };

    bool IsPathLocked(const std::string& lane_name, std::string path_name)
    {
        bool isLocked = false;
        if (IsPathGlobal(path_name))
        {
            for (auto paths : m_lanesLocks)
            {
                auto lockIterator = paths.second.find(path_name);
                if (lockIterator != paths.second.end())
                {
                    isLocked |= lockIterator->second;
                }
            }
        }
        else
        {
            auto paths = m_lanesLocks.find(lane_name);
            if (paths == m_lanesLocks.end())
            {
                return false;
            }
            auto pathIterator = paths->second.find(path_name);
            if (pathIterator == paths->second.end())
            {
                return false;
            }
            isLocked |= pathIterator->second;
        }
        auto pathDependency = PathDependency(path_name);
        if (!isLocked && pathDependency)
        {
            isLocked |= IsPathLocked(lane_name, *pathDependency);
        }
        return isLocked;
    }

    void SetPathLock(const std::string& lane_name, std::string path_name, bool lock)
    {
        auto pathsIterator = m_lanesLocks.find(lane_name);
        if (pathsIterator == m_lanesLocks.end())
        {
            return;
        }
        auto pathLockIterator = pathsIterator->second.find(path_name);
        if (pathLockIterator == pathsIterator->second.end())
        {
            return;
        }
        pathLockIterator->second = lock;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>("orchestrator", rclcpp::NodeOptions());

    LockService lockService(node);
    lockService.Initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
