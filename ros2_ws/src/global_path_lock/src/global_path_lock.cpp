#include <functional>
#include <lock_service_msgs/srv/lock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class LockService
{
    using PathLocks = std::set<std::string>;

public:
    LockService(rclcpp::Node::SharedPtr node)
    {
        m_service = node->create_service<lock_service_msgs::srv::Lock>(
            "/lock_service", std::bind(&LockService::CallLock, this, std::placeholders::_1, std::placeholders::_2));
    };

    void CallLock(
        const std::shared_ptr<lock_service_msgs::srv::Lock::Request> request,
        std::shared_ptr<lock_service_msgs::srv::Lock::Response> response)
    {
        auto key = request->key;
        bool askingForLock = request->lock_status;
        auto wasCurrentlyLocked = (m_locks.count(key) > 0);
        if (askingForLock)
        {   // lock requested
            if (!wasCurrentlyLocked)
            {   // lock it!
                m_locks.insert(key);
            }
            response->result = !wasCurrentlyLocked;
            return;
        }

        if (!askingForLock)
        {   // unlock requested
            if (wasCurrentlyLocked)
            {	// remove lock
                m_locks.erase(key);
            }
            // regardless whether there was lock or not, it is always ok to call unlock
            response->result = true;
            return;
        }
    };

private:
    PathLocks m_locks;
    rclcpp::Service<lock_service_msgs::srv::Lock>::SharedPtr m_service;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("orchestrator");
    LockService lockService(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
