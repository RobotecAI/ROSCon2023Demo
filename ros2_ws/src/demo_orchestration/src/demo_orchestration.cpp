
#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "ur_moveit_demo_msg/action/mtc.hpp"

class MTCActionClient
{
public:
    using MTCAction = ur_moveit_demo_msg::action::Mtc;
    using GoalHandleMTC = rclcpp_action::ClientGoalHandle<MTCAction>;

    MTCActionClient(rclcpp::Node::SharedPtr node, std::mutex& lock, std::string ur_namespace)
        : node_(node), lock_(lock)
    {
        this->client_ptr_ = rclcpp_action::create_client<MTCAction>(node_, "/" + ur_namespace + "/MTC");
    }

    void send_goal(int number_of_boxes)
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
    }

private:
    rclcpp_action::Client<MTCAction>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_;

    std::mutex& lock_;

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
        lock_.unlock();
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
    using Nav2Action = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;

    Nav2ActionClient(rclcpp::Node::SharedPtr node, std::mutex& lock, std::string amr_ns)
        : node_(node), lock_(lock)
    {
        this->client_ptr_ = rclcpp_action::create_client<Nav2Action>(node_, "/" + amr_ns + "/navigate_to_pose");
    }

    void send_goal(geometry_msgs::msg::PoseStamped targetPose)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = Nav2Action::Goal();

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        goal_msg.pose = targetPose;

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

    std::mutex& lock_;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
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
        lock_.unlock();
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);


    std::mutex lock;
    lock.lock();

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
    auto amr_ns = node->get_parameter("amr_namespace").as_string();
    auto number_of_boxes = node->get_parameter("number_of_boxes").as_int();

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto PickupPosition = tf_buffer_->lookupTransform(ur_ns + "/odom", ur_ns + "/Pickup", node->get_clock()->now());
    auto DropPosition = tf_buffer_->lookupTransform(ur_ns +  "/odom", ur_ns + "/Drop", node->get_clock()->now());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::PoseStamped targetPosePickup;
    targetPosePickup.pose.position.set__x(PickupPosition.transform.translation.x).set__y(PickupPosition.transform.translation.y).set__z(PickupPosition.transform.translation.y);
    targetPosePickup.header.frame_id = "ur1/odom";
    targetPosePickup.pose.orientation.set__w(PickupPosition.transform.rotation.w).set__x(PickupPosition.transform.rotation.x).set__y(PickupPosition.transform.rotation.y).set__z(PickupPosition.transform.rotation.z);

    geometry_msgs::msg::PoseStamped targetPoseDrop;
    targetPoseDrop.pose.position.set__x(DropPosition.transform.translation.x).set__y(DropPosition.transform.translation.y).set__z(DropPosition.transform.translation.y);
    targetPoseDrop.header.frame_id = "ur1/odom";
    targetPoseDrop.pose.orientation.set__w(DropPosition.transform.rotation.w).set__x(DropPosition.transform.rotation.x).set__y(DropPosition.transform.rotation.y).set__z(DropPosition.transform.rotation.z);
    
    auto moveitClient = std::make_shared<MTCActionClient>(node, lock, ur_ns);
    auto amrClient = std::make_shared<Nav2ActionClient>(node, lock, amr_ns);
    while(true) {
        amrClient->send_goal(targetPosePickup);
        lock.lock();
        moveitClient->send_goal(number_of_boxes);
        lock.lock();
        amrClient->send_goal(targetPoseDrop);
        lock.lock();
    }

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
