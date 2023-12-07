#include "taskConstructor.h"

#include <pilz_industrial_motion_planner/planning_context_ptp.h>
#include "utils.h"

namespace Palletization
{
    RoboticArmController::RoboticArmController(
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroupInterface, const std::string& ns)
            : m_moveGroupInterface(moveGroupInterface)
    {
        // Poses below were set arbitrarily, based on the scene setup.

        PredefinePoseJointSpace pickupConfig;
        pickupConfig.insert({ns + "/wrist_1_joint", -0.8385483622550964});
        pickupConfig.insert({ns + "/wrist_2_joint", 1.5643877983093262});
        pickupConfig.insert({ns + "/elbow_joint", -1.550349235534668});
        pickupConfig.insert({ns + "/shoulder_pan_joint", -2.7139534950256348});
        pickupConfig.insert({ns + "/shoulder_lift_joint", -2.314471483230591});
        pickupConfig.insert({ns + "/wrist_1_joint", -0.8385483622550964});

        PredefinePoseJointSpace liftConfig;
        liftConfig.insert({ns + "/wrist_1_joint", -1.6993759870529175});
        liftConfig.insert({ns + "/elbow_joint", -1.0284128189086914});
        liftConfig.insert({ns + "/shoulder_lift_joint", -1.9712634086608887});
        liftConfig.insert({ns + "/wrist_2_joint", 1.5634684562683105});
        liftConfig.insert({ns + "/shoulder_pan_joint", -2.644500255584717});
        liftConfig.insert({ns + "/wrist_3_joint", -4.113039656});

        PredefinePoseJointSpace dropConfig;
        dropConfig.insert({ns + "/wrist_1_joint", -1.5789473056793213});
        dropConfig.insert({ns + "/elbow_joint", -0.9531064033508301});
        dropConfig.insert({ns + "/shoulder_lift_joint", -1.8});
        dropConfig.insert({ns + "/wrist_2_joint", 1.5672444105148315});
        dropConfig.insert({ns + "/shoulder_pan_joint", -0.15679530799388885});
        dropConfig.insert({ns + "/wrist_3_joint", -3.1959822177886963});

        m_predefinedPoses.insert({PickupPoseName, pickupConfig});
        m_predefinedPoses.insert({LiftPoseName, liftConfig});
        m_predefinedPoses.insert({DropPoseName, dropConfig});

        this->m_ns = ns;
    }


    bool RoboticArmController::SetPosePIP(const std::string &poseName)
    {
        m_moveGroupInterface->setMaxAccelerationScalingFactor(0.5f);
        m_moveGroupInterface->setMaxVelocityScalingFactor(1.0f);

        m_moveGroupInterface->setPlanningPipelineId("pilz_industrial_motion_planner");
        m_moveGroupInterface->setPlannerId("PTP");
        m_moveGroupInterface->setJointValueTarget(m_predefinedPoses.at(poseName));

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode errorCode;
        if ((errorCode = m_moveGroupInterface->plan(plan)) != moveit::core::MoveItErrorCode::SUCCESS ||
            (errorCode = m_moveGroupInterface->execute(plan)) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to execute move plan for pose " << poseName << " (error code: " << moveit::core::error_code_to_string(errorCode) << ")");
            return false;
        }
        return true;
    }

    bool RoboticArmController::SetPosePIP(
        const Eigen::Vector3d& tcpPosition, const Eigen::Quaterniond& tcpOrientation, float speed, const std::string& interpolation)
    {
        Eigen::Isometry3d tcpPose;
        tcpPose.fromPositionOrientationScale(tcpPosition, tcpOrientation, Eigen::Vector3d::Ones());
        m_moveGroupInterface->setMaxAccelerationScalingFactor(0.5f);
        m_moveGroupInterface->setMaxVelocityScalingFactor(speed);
        m_moveGroupInterface->setPlanningPipelineId("pilz_industrial_motion_planner");
        m_moveGroupInterface->setPlannerId(interpolation);
        m_moveGroupInterface->setApproximateJointValueTarget(tcpPose, m_ns + "/gripper_link");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode errorCode;
        if ((errorCode = m_moveGroupInterface->plan(plan)) != moveit::core::MoveItErrorCode::SUCCESS ||
            (errorCode = m_moveGroupInterface->execute(plan)) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to execute move plan (error code: " << moveit::core::error_code_to_string(errorCode) << ")");
            return false;
        }
        return true;
    }

    Eigen::Quaterniond RoboticArmController::GetCurrentOrientation() const
    {
        const auto currentPose = m_moveGroupInterface->getCurrentPose(m_ns + "/gripper_link");
        Eigen::Quaterniond currentOrientation(currentPose.pose.orientation.w, currentPose.pose.orientation.x,
                                              currentPose.pose.orientation.y, currentPose.pose.orientation.z);
        return currentOrientation;
    }

} // namespace TaskConstructor
