#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>
#include <blind_path_follower_msgs/action/follow_path.hpp>
#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/create_server.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

//! Interpolate between multiple poses
//! @param p 0-1 value of interpolation
//! @param poses vector of poses to interpolate between
//! @return interpolated pose
Eigen::Affine3d interpolatePath(double p, const std::vector<Eigen::Affine3d>& poses)
{
    const double indexUnormalized = poses.size() * p;
    int indexLower = floor(indexUnormalized);
    int indexUpper = ceil(indexUnormalized);
    indexLower = std::min(std::max(0, indexLower), int(poses.size() - 1));
    indexUpper = std::min(std::max(0, indexUpper), int(poses.size() - 1));
    if (indexLower == indexUpper)
    {
        return poses[indexLower];
    }
    auto& poseLower = poses[indexLower];
    auto& poseUpper = poses[indexUpper];
    const double fraction = indexUnormalized - indexLower;
    assert(fraction >= 0.0 && fraction <= 1.0);

    Eigen::Affine3d poseInterpolated = Eigen::Affine3d::Identity();
    poseInterpolated.translation() = poseLower.translation() + fraction * (poseUpper.translation() - poseLower.translation());
    const Eigen::Quaterniond qLower(poseLower.rotation());
    const Eigen::Quaterniond qUpper(poseUpper.rotation());
    poseInterpolated.rotate(qLower.slerp(fraction, qUpper));

    return poseInterpolated;
}

//! Get the signed angle between two vectors
//! @param v1 first vector
//! @param v2 second vector
//! @return angle between the vectors
float GetAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    return atan2(v1.cross(v2).dot(Eigen::Vector3d::UnitZ()), v1.dot(v2));
}

double getPathLength(const std::vector<Eigen::Affine3d>& poses)
{
    double length = 0.0;
    for (size_t i = 1; i < poses.size(); ++i)
    {
        length += (poses[i].translation() - poses[i - 1].translation()).norm();
    }
    return length;
}

class FollowPathActionServer
{
public:
    using FlwPthAction = blind_path_follower_msgs::action::FollowPath;
    using FlwPthGoal = rclcpp_action::ServerGoalHandle<FlwPthAction>;

    FollowPathActionServer(rclcpp::Node::SharedPtr node, std::string ns)
        : node_(node)
        , tf_buffer_(node->get_clock())
        , tf_listener_(tf_buffer_)
    {
        using namespace std::placeholders;

        goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("blind_goal", 1);
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(int(1000 * LoopTimeSec)), std::bind(&FollowPathActionServer::timer_callback, this));
        timer_->cancel();
        cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        if (!ns.empty() && ns[0] == '/')
        {
            ns_ = ns.substr(1);
        }
        else
        {
            ns_ = ns;
        }
        if (ns.empty() || ns[ns.size() - 1] != '/')
        {
            RCLCPP_WARN(node->get_logger(), "Starting path follower, but with empty namespace. This is probably not what you want.");
        }

        this->action_server_ = rclcpp_action::create_server<FlwPthAction>(
            node->get_node_base_interface(),
            node->get_node_clock_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            "blind_follow_path",
            std::bind(&FollowPathActionServer::handle_goal, this, _1, _2),
            std::bind(&FollowPathActionServer::handle_cancel, this, _1),
            std::bind(&FollowPathActionServer::handle_accepted, this, _1));
    }

private:
    constexpr static double LoopTimeSec = 0.01;
    rclcpp_action::Server<FlwPthAction>::SharedPtr action_server_;
    rclcpp::Node::SharedPtr node_;
    std::string ns_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_; //! < Publishes the goal pose
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<FlwPthGoal> goal_handle_;
    double desired_linear_velocity_ = 0;
    bool reverse_ = false;
    double path_length_ = 0;
    double path_elapsed_ = 0;
    bool should_stop = false;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Eigen::Affine3d> poses_;

    bool m_robotPassedStart = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr objectDetector;

    rclcpp_action::GoalResponse handle_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const FlwPthAction::Goal> goal)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel([[maybe_unused]] const std::shared_ptr<FlwPthGoal> goal_handle)
    {
        objectDetector.reset();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<FlwPthGoal> goal_handle)
    {
        using namespace std::placeholders;
        goal_handle_ = goal_handle;
        desired_linear_velocity_ = goal_handle->get_goal()->speed;
        reverse_ = goal_handle->get_goal()->reverse;
        poses_.clear();

        objectDetector = node_->create_subscription<std_msgs::msg::Bool>(
            "/object_detector",
            10,
            [&](std_msgs::msg::Bool msg)
            {
                should_stop = msg.data;
            });

        for (auto& pose : goal_handle->get_goal()->poses)
        {
            Eigen::Affine3d poseEigen;
            tf2::fromMsg(pose.pose, poseEigen);
            poses_.push_back(poseEigen);
        }
        path_length_ = getPathLength(poses_);
        path_elapsed_ = 0;
        timer_->reset();
        RCLCPP_INFO(
            node_->get_logger(),
            "Received tack to follow, number of goals %ld, length %f",
            goal_handle->get_goal()->poses.size(),
            path_length_);
    }

    void timer_callback()
    {
        double DesiredLinearVelocity = desired_linear_velocity_;
        const double MaxBearingError = M_PI_2 * 0.2; //!< For larger bearing deviation we stop linear motion
        const double MaxVirtualDistance = 0.3; //!< Distance between a the virtual robot and the real robot before slowdown.
        const int LookupTime = 100; //!< Multiplier to detect sharp turns
        const double CrossTrackGain = 0.5; //!< How much to correct angular velocity for cross track error (distance to track)
        const double StartTolerance = 0.3; //!< Distance between virtual robot and real robot before start
        const double AlongTrackGain = 0.5; //!< How much to correct linear velocity for along track error (distance along track)
        const double BearingGain = 2.0; //!< How much to correct angular velocity for bearing error (angle to track)
        const double MaxAngularSpeed = 0.6;
        if (poses_.empty())
        {
            geometry_msgs::msg::Twist cmd;
            cmd_publisher_->publish(cmd);
            timer_->cancel();
            auto result = std::make_shared<FlwPthAction::Result>();
            result->success = true;
            m_robotPassedStart = false;
            objectDetector.reset();
            goal_handle_->succeed(result);
            return;
        }

        const double p = DesiredLinearVelocity * path_elapsed_ / path_length_;
        const double pFuture = std::min(1., DesiredLinearVelocity * (path_elapsed_ + LoopTimeSec * LookupTime) / path_length_);
        const Eigen::Affine3d idealGoal = interpolatePath(p, poses_);
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.pose = tf2::toMsg(idealGoal);
        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = node_->now();
        goal_publisher_->publish(poseMsg);
        Eigen::Affine3d poseBaseLink = Eigen::Affine3d::Identity();

        try
        {
            const geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform("map", ns_ + "/base_link", tf2::TimePointZero);
            poseBaseLink = tf2::transformToEigen(transformStamped.transform);
        } catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
            return;
        }

        if (reverse_)
        {
            poseBaseLink = poseBaseLink * Eigen::AngleAxis<double>(M_PI, Eigen::Vector3d(0.0, 0.0, 1.0));
        }

        if ((poses_[0].translation() - poseBaseLink.translation()).norm() < StartTolerance)
        {
            m_robotPassedStart = true;
        }

        const Eigen::Vector3d robotLocationInGoalSpace = idealGoal.inverse() * poseBaseLink.translation();

        const double crossTrackError = -robotLocationInGoalSpace.y();
        const double alongTrackError = -robotLocationInGoalSpace.x();

        const Eigen::Vector3d tangentGoal = idealGoal.rotation().col(0); // get X axis of goal frame
        const Eigen::Vector3d tangentRobot = poseBaseLink.rotation().col(0); // get X axis of robot frame

        const float bearingError = GetAngle(tangentRobot, tangentGoal);

        geometry_msgs::msg::Twist cmd;
        if (!m_robotPassedStart)
        {
            // if robot is not close to the start, adjust to the first point of spline
            double closeness = (poses_[0].translation() - poseBaseLink.translation()).norm();
            double adjustableBearingGain = BearingGain * std::min(1., closeness * 0.25);
            double adjustableCrossTrackGain = CrossTrackGain * std::max(1.5, 0.25 / closeness);
            cmd.angular.z = bearingError * adjustableBearingGain + crossTrackError * adjustableCrossTrackGain;
        }
        else
        {
            // if robot is on the spline, adjust to the spline
            cmd.angular.z = bearingError * BearingGain + crossTrackError * CrossTrackGain;
        }

        // limit angular speed
        cmd.angular.z = std::max(std::min(cmd.angular.z, MaxAngularSpeed), -MaxAngularSpeed);

        // adjust linear speed to the position on the spline of the virtual robot
        double requestedLinearVelocity = DesiredLinearVelocity + alongTrackError * AlongTrackGain;

        // if the bearing error is too large, stop linear movement
        if (std::abs(bearingError) > MaxBearingError || should_stop)
        {
            requestedLinearVelocity = 0.0;
        }
        else
        {
            requestedLinearVelocity *= std::abs(
                (MaxBearingError - std::abs(m_robotPassedStart ? (bearingError) : (bearingError + crossTrackError))) / MaxBearingError);
        }

        // if the sharp turn is detected in the future, adapt the linear speed
        auto futureAngle = std::abs(GetAngle(interpolatePath(pFuture, poses_).rotation().col(0), tangentRobot));

        if (futureAngle > MaxBearingError && m_robotPassedStart)
        {
            requestedLinearVelocity /= 2.0; // slow down if we are going to turn
        }

        if (!m_robotPassedStart)
        {
            // if the robot is not close to the start, slowly move to the start
            cmd.linear.x = DesiredLinearVelocity * 0.5;
        }
        else
        {
            // if the robot is on the spline, move with the requested speed
            cmd.linear.x = requestedLinearVelocity;
        }

        if (reverse_)
        {
            cmd.linear.x *= -1;
        }

        auto feedback = std::make_shared<FlwPthAction::Feedback>();
        feedback->progress = p;
        goal_handle_->publish_feedback(feedback);

        if (p >= 1.0)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            timer_->cancel();
            auto result = std::make_shared<FlwPthAction::Result>();
            result->success = true;
            m_robotPassedStart = false;
            objectDetector.reset();
            std::cout << "Path Succeed" << std::endl;
            goal_handle_->succeed(result);
        }
        cmd_publisher_->publish(cmd);
        if (m_robotPassedStart && !should_stop)
        {
            if (std::abs(bearingError) > MaxBearingError)
            {
                path_elapsed_ += LoopTimeSec / 10;
            }
            else if (futureAngle > MaxBearingError || (idealGoal.translation() - poseBaseLink.translation()).norm() > MaxVirtualDistance)
            {
                path_elapsed_ += LoopTimeSec / 2;
            }
            else
            {
                path_elapsed_ += LoopTimeSec;
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("follow_path", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto ns = node->get_namespace();
    RCLCPP_INFO(node->get_logger(), "Starting path follower");

    auto action_server = std::make_shared<FollowPathActionServer>(node, ns);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
