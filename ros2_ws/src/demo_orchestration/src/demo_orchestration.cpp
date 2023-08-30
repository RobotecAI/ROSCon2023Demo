
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/detail/navigate_through_poses__struct.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <vector>

#include "ur_moveit_demo_msg/action/mtc.hpp"

int num_of_boxes;

enum AMRState
{
    MovingToPickup,
    MovingToDrop,
    MovingToPark,
    Drop,
    Pickup,
    Park,
};

enum MoveItState
{
    Load,
    Ready,
    AfterLoad,
    BeforeLoad,
};

class MTCActionClient
{
public:
    using MTCAction = ur_moveit_demo_msg::action::Mtc;
    using GoalHandleMTC = rclcpp_action::ClientGoalHandle<MTCAction>;

    MTCActionClient(rclcpp::Node::SharedPtr node, std::string ur_namespace)
        : node_(node)
    {
        this->client_ptr_ = rclcpp_action::create_client<MTCAction>(node_, "/" + ur_namespace + "/MTC");
    }

    void send_goal(int number_of_boxes, std::function<void()> callback)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MTCAction::Goal();
        goal_msg.num_of_boxes = number_of_boxes;

        RCLCPP_INFO(node_->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MTCAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MTCActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&MTCActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&MTCActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        m_resultCallback = callback;
    }

private:
    rclcpp_action::Client<MTCAction>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_;

    std::function<void()> m_resultCallback;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<ur_moveit_demo_msg::action::Mtc>> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleMTC::SharedPtr, const std::shared_ptr<const MTCAction::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Placed box number: ";
        ss << feedback->current_box << " ";

        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleMTC::WrappedResult& result)
    {
        m_resultCallback();
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";

        ss << result.result->success << " ";

        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }
}; // class MTCActionClient

class Nav2ActionClient
{
public:
    using Nav2Action = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;

    Nav2ActionClient(rclcpp::Node::SharedPtr node, std::string amr_ns)
        : node_(node)
    {
        this->client_ptr_ = rclcpp_action::create_client<Nav2Action>(node_, "/" + amr_ns + "/navigate_through_poses");
    }

    void send_goal(nav_msgs::msg::Path targetPath, std::function<void()> callback)
    {
        m_resultCallback = callback;

        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Nav2Action::Goal();

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        goal_msg.poses = targetPath.poses;

        RCLCPP_INFO(node_->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&Nav2ActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Nav2ActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Nav2ActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Nav2Action>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_;

    std::function<void()> m_resultCallback;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleNav2::SharedPtr, const std::shared_ptr<const GoalHandleNav2::Feedback> feedback)
    {
    }

    void result_callback(const GoalHandleNav2::WrappedResult& result)
    {
        m_resultCallback();
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal nav was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            return;
        }
    }
};

geometry_msgs::msg::PoseStamped TransformToPose(geometry_msgs::msg::TransformStamped transform)
{
    geometry_msgs::msg::PoseStamped result;
    result.pose.position.set__x(transform.transform.translation.x)
        .set__y(transform.transform.translation.y)
        .set__z(transform.transform.translation.y);
    result.header.frame_id = transform.header.frame_id;
    result.pose.orientation.set__w(transform.transform.rotation.w)
        .set__x(transform.transform.rotation.x)
        .set__y(transform.transform.rotation.y)
        .set__z(transform.transform.rotation.z);

    return result;
}

struct AMRController
{
    AMRState state;
    std::shared_ptr<Nav2ActionClient> client;
    std::string amrNamespace;
};

struct MoveItController
{
    MoveItState state;
    std::shared_ptr<MTCActionClient> client;
};

class StationOrchestrator
{
public:
    StationOrchestrator(rclcpp::Node::SharedPtr node, std::string moveItNamespace, std::vector<std::string> amrNamespaces)
    {
        m_moveItController = {
            MoveItState::Ready,
            std::make_shared<MTCActionClient>(node, moveItNamespace),
        };
        for (auto amrNamespace : amrNamespaces)
        {
            m_amrControllers.push_back({
                AMRState::Park,
                std::make_shared<Nav2ActionClient>(node, amrNamespace),
                amrNamespace,
            });
        }

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        m_posePickup =
            TransformToPose(tf_buffer_->lookupTransform(moveItNamespace + "/odom", moveItNamespace + "/Pickup", node->get_clock()->now()));
        m_poseDrop =
            TransformToPose(tf_buffer_->lookupTransform(moveItNamespace + "/odom", moveItNamespace + "/Drop", node->get_clock()->now()));
        m_posePark =
            TransformToPose(tf_buffer_->lookupTransform(moveItNamespace + "/odom", moveItNamespace + "/Park", node->get_clock()->now()));

        rclcpp::QoS qos(5);
        qos.transient_local();

        auto pickupPathSubscriber = node->create_subscription<nav_msgs::msg::Path>(
            "/pickupPath",
            qos,
            [&](nav_msgs::msg::Path path)
            {
                m_pickupPath = path;
            });

        auto parkSubscriber = node->create_subscription<nav_msgs::msg::Path>(
            "/parkPath",
            qos,
            [&](nav_msgs::msg::Path path)
            {
                m_parkPath = path;
            });

        auto dropSubscriber = node->create_subscription<nav_msgs::msg::Path>(
            "/dropPath",
            qos,
            [&](nav_msgs::msg::Path path)
            {
                m_dropPath = path;
            });

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void start()
    {
        std::thread runner(
            [&]()
            {
                while (true)
                {
                    {
                        std::lock_guard<std::mutex> notifyLock(m_notifyLock);
                        for (std::vector<AMRController>::size_type i = 0; i < m_amrControllers.size(); i++)
                        {
                            auto& m_amrController = m_amrControllers[i];
                            if (m_amrController.state == AMRState::Drop)
                            {
                                m_amrController.state = AMRState::MovingToPark;
                                auto boundFn =
                                    std::bind(&StationOrchestrator::notifyAMR, this, AMRState::Park, m_amrController.amrNamespace);
                                m_amrController.client->send_goal(
                                    m_parkPath,
                                    std::bind(&StationOrchestrator::notifyAMR, this, AMRState::Park, m_amrController.amrNamespace));
                            }
                        }
                        if (m_moveItController.state == MoveItState::BeforeLoad)
                        {
                            for (std::vector<AMRController>::size_type i = 0; i < m_amrControllers.size(); i++)
                            {
                                auto& m_amrController = m_amrControllers[i];
                                if (m_amrController.state == AMRState::Pickup)
                                {
                                    m_moveItController.state = MoveItState::Load;
                                    // m_amrController.client->send_goal(
                                    //     m_posePickup,
                                    //     std::bind(&StationOrchestrator::notifyAMR, this, AMRState::Pickup, m_amrController.amrNamespace));
                                    m_moveItController.client->send_goal(num_of_boxes, std::bind(&StationOrchestrator::notifyMoveIt, this, MoveItState::AfterLoad));
                                    break;
                                }
                            }
                        }
                        if (m_moveItController.state == MoveItState::AfterLoad)
                        {
                            for (std::vector<AMRController>::size_type i = 0; i < m_amrControllers.size(); i++)
                            {
                                auto& m_amrController = m_amrControllers[i];
                                if (m_amrController.state == AMRState::Pickup)
                                {
                                    m_amrController.state = AMRState::MovingToDrop;
                                    m_moveItController.state = MoveItState::Ready;
                                    m_amrController.client->send_goal(
                                        m_dropPath,
                                        std::bind(&StationOrchestrator::notifyAMR, this, AMRState::Drop, m_amrController.amrNamespace));
                                    break;
                                }
                            }
                        }
                        if (m_moveItController.state == MoveItState::Ready)
                        {
                            for (std::vector<AMRController>::size_type i = 0; i < m_amrControllers.size(); i++)
                            {
                                auto& m_amrController = m_amrControllers[i];
                                if (m_amrController.state == AMRState::Park)
                                {
                                    m_amrController.state = AMRState::MovingToPickup;
                                    m_moveItController.state = MoveItState::BeforeLoad;
                                    m_amrController.client->send_goal(
                                        m_pickupPath,
                                        std::bind(&StationOrchestrator::notifyAMR, this, AMRState::Pickup, m_amrController.amrNamespace));
                                    break;
                                }
                            }
                        }
                    }

                    m_lock.lock();
                }
            });
        runner.join();
    }

    void notifyAMR(AMRState amrState, std::string amrNamespace)
    {
        std::lock_guard<std::mutex> lock(m_notifyLock);
        for (std::vector<AMRController>::size_type i = 0; i < m_amrControllers.size(); i++)
        {
            if (m_amrControllers[i].amrNamespace == amrNamespace)
            {
                m_amrControllers[i].state = amrState;
            }
        }
        m_lock.unlock();
    }

    void notifyMoveIt(MoveItState moveItState)
    {
        std::lock_guard<std::mutex> lock(m_notifyLock);
        m_moveItController.state = moveItState;
        m_lock.unlock();
    }

private:
    MoveItController m_moveItController;
    std::vector<AMRController> m_amrControllers;

    geometry_msgs::msg::PoseStamped m_posePickup;
    geometry_msgs::msg::PoseStamped m_poseDrop;
    geometry_msgs::msg::PoseStamped m_posePark;

    nav_msgs::msg::Path m_pickupPath;
    nav_msgs::msg::Path m_dropPath;
    nav_msgs::msg::Path m_parkPath;

    std::mutex m_lock;
    std::mutex m_notifyLock;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto const node =
        std::make_shared<rclcpp::Node>("demo_orchestration", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread(
        [&executor]()
        {
            executor.spin();
        });

    auto ur_ns = node->get_parameter("ur_namespace").as_string();
    auto amr_namespaces = node->get_parameter("amr_namespaces").as_string_array();
    num_of_boxes = node->get_parameter("number_of_boxes").as_int();

    StationOrchestrator stationOrchestrator(node, ur_ns, amr_namespaces);
    stationOrchestrator.start();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
