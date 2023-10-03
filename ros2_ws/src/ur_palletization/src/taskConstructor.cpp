#include "taskConstructor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "utils.h"
#include "pilz_industrial_motion_planner/planning_context_ptp.h"

namespace Palletization
{

    RoboticArmController::RoboticArmController(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface, std::string ns)
        : m_move_groupIterface(moveGroupInterface)
    {
        PredefinePoseJointSpace pickupConfig;
        pickupConfig.insert({ ns + "/wrist_1_joint", -0.8385483622550964 });
        pickupConfig.insert({ ns + "/wrist_2_joint", 1.5643877983093262 });
        pickupConfig.insert({ ns + "/elbow_joint", -1.550349235534668 });
        pickupConfig.insert({ ns + "/shoulder_pan_joint", -2.7139534950256348 });
        pickupConfig.insert({ ns + "/shoulder_lift_joint", -2.314471483230591 });
        pickupConfig.insert({ ns + "/wrist_1_joint", -0.8385483622550964 });

        PredefinePoseJointSpace liftConfig;
        liftConfig.insert({ ns + "/wrist_1_joint", -1.6993759870529175 });
        liftConfig.insert({ ns + "/elbow_joint", -1.0284128189086914 });
        liftConfig.insert({ ns + "/shoulder_lift_joint", -1.9712634086608887 });
        liftConfig.insert({ ns + "/wrist_2_joint", 1.5634684562683105 });
        liftConfig.insert({ ns + "/shoulder_pan_joint", -2.644500255584717 });
        liftConfig.insert({ ns + "/wrist_3_joint", -4.113039656 });

        PredefinePoseJointSpace dropConfig;
        dropConfig.insert({ ns + "/wrist_1_joint", -1.5789473056793213 });
        dropConfig.insert({ ns + "/elbow_joint", -0.9531064033508301 });
        dropConfig.insert({ ns + "/shoulder_lift_joint", -1.8 });
        dropConfig.insert({ ns + "/wrist_2_joint", 1.5672444105148315 });
        dropConfig.insert({ ns + "/shoulder_pan_joint", -0.15679530799388885 });
        dropConfig.insert({ ns + "/wrist_3_joint", -3.1959822177886963 });

        m_predefinedPoses.insert({ PickupPoseName, pickupConfig });
        m_predefinedPoses.insert({ "lift", liftConfig });
        m_predefinedPoses.insert({ "drop", dropConfig });

        this->ns = ns;
    }


    bool RoboticArmController::setPosePIP(const std::string& poseName)
    {
        m_move_groupIterface->setMaxAccelerationScalingFactor(0.5f);
        m_move_groupIterface->setMaxVelocityScalingFactor(1.0f);

        m_move_groupIterface->setPlanningPipelineId ("pilz_industrial_motion_planner");
        m_move_groupIterface->setPlannerId("PTP");
        m_move_groupIterface->setJointValueTarget(m_predefinedPoses.at(poseName));
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto isOk = m_move_groupIterface->plan(plan);
        if (isOk)
        {
            m_move_groupIterface->execute(plan);
            return true;
        }
        else
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning failed for pose " << poseName);
            return false;
        }
    }

    bool RoboticArmController::setPosePIP(const Eigen::Vector3d &tcp_position, const Eigen::Quaterniond& tcp_orientation, float speed, const std::string& interpolation)
    {

        Eigen::Isometry3d tcp_pose;
        tcp_pose.fromPositionOrientationScale(tcp_position, tcp_orientation, Eigen::Vector3d::Ones());
        m_move_groupIterface->setMaxAccelerationScalingFactor(0.5f);
        m_move_groupIterface->setMaxVelocityScalingFactor(speed);
        m_move_groupIterface->setPlanningPipelineId ("pilz_industrial_motion_planner");
        m_move_groupIterface->setPlannerId("PTP");
        m_move_groupIterface->setApproximateJointValueTarget(tcp_pose, ns + "/gripper_link");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto isOk = m_move_groupIterface->plan(plan);
        if (!isOk)
        {
            return false;
        }
        auto successExecution = m_move_groupIterface->execute(plan);

        if (!successExecution)
        {
            return false;
        }
        return true;
    }


    Eigen::Quaterniond RoboticArmController::getCurrentOrientation()
    {
        auto currentPose = m_move_groupIterface->getCurrentPose(ns + "/gripper_link");
        Eigen::Quaterniond currentOrientation(currentPose.pose.orientation.w, currentPose.pose.orientation.x,
                                              currentPose.pose.orientation.y, currentPose.pose.orientation.z);
        return currentOrientation;
    }


} // namespace TaskConstructor
