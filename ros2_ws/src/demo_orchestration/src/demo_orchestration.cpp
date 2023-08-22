
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

#include "ur_moveit_demo_msg/action/mtc.hpp"

class MTCActionClient
{
public:
    using MTCAction = ur_moveit_demo_msg::action::Mtc;
    using GoalHandleMTC = rclcpp_action::ClientGoalHandle<MTCAction>;

    MTCActionClient(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        this->client_ptr_ = rclcpp_action::create_client<MTCAction>(node_, "MTC");


        this->timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MTCActionClient::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MTCAction::Goal();
        goal_msg.num_of_boxes = 3;

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

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto ParkPosition = tf_buffer_->lookupTransform("ur1/odom", "ur1/Park", node->get_clock()->now());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::stringstream ss;
    ss << ParkPosition.transform.translation.x;
    RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/otto_1/goal_pose", rclcpp::SystemDefaultsQoS());
    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.pose.position.set__x(ParkPosition.transform.translation.x).set__y(ParkPosition.transform.translation.y).set__z(ParkPosition.transform.translation.y);
    targetPose.header.frame_id = "ur1/odom";
    targetPose.pose.orientation.set__w(0.707).set__z(-0.707);

    publisher->publish(targetPose);

    auto client = std::make_shared<MTCActionClient>(node);
    client->send_goal();

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
