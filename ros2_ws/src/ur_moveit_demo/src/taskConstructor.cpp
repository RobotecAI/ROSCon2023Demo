#include "taskConstructor.h"
#include "moveit/task_constructor/solvers/multi_planner.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "utils.h"
#include "pilz_industrial_motion_planner/planning_context_ptp.h"

namespace TaskConstructor
{

    MTCController::MTCController(rclcpp::Node::SharedPtr node, std::string ns)
        : node_(node)
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

        robot_model_loader::RobotModelLoader robot_model_loader(node);
        m_kinematic_model = robot_model_loader.getModel();
        moveit::planning_interface::MoveGroupInterface::Options options(ns + "/ur_manipulator", robot_model_loader.getRobotDescription(), "/" + ns);
        m_move_groupIterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options);
        this->ns = ns;
    }

    bool MTCController::doTask(mtc::Task& task)
    {
        task_ = std::move(task);
        try
        {
            task_.init();
        } catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e);
            return false;
        }
	     RCLCPP_INFO_STREAM(node_->get_logger(), "Task initialized: " << task_.name());
        try
        {
            if (!task_.plan(3))
            {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "Task planning failed for task " << task_.name());
			       task_.explainFailure();
                return false;
            }

        } catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e);
            return false;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Task execution failed");
            return false;
        }

        return true;
    }
    bool MTCController::setPosePIP(const std::string& poseName)
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

    bool MTCController::setPosePIP(const Eigen::Vector3d &tcp_position, float speed, const std::string& interpolation)
    {

        const Eigen::Quaterniond OrientationDown{ -0.5, -0.5, 0.5, -0.5 };
        Eigen::Isometry3d tcp_pose;
        tcp_pose.fromPositionOrientationScale(tcp_position, OrientationDown, Eigen::Vector3d::Ones());
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
    mtc::Task MTCController::createTaskGrab(const geometry_msgs::msg::Pose& boxPose, std::string boxname, std::string ns)
    {
        // ToDo make use of boxPose parameter
        mtc::Task task(ns);
        task.stages()->setName("Grab box");
        task.loadRobotModel(node_);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(.005);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        sampling_planner->setTimeout(10.);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);
	    interpolation_planner->setMaxVelocityScalingFactor(0.5);
	    interpolation_planner->setMaxAccelerationScalingFactor(0.5);

	     auto multi_planner = std::make_shared<mtc::solvers::MultiPlanner>();
	     multi_planner->push_back(interpolation_planner);
	     multi_planner->push_back(sampling_planner);

        {
            auto PickupSerial = std::make_unique<mtc::SerialContainer>("Pickup");
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Set scene");
                stage->addObject(Utils::CreateBoxCollision(
                    boxname, BoxDimension, Utils::fromMsgPosition(boxPose.position), Eigen::Quaterniond::Identity(), ns));
                stage->allowCollisions(
                    boxname,
                    task.getRobotModel()->getJointModelGroup(ns + "/ur_manipulator")->getLinkModelNamesWithCollisionGeometry(),
                    false);
                PickupSerial->insert(std::move(stage));
            }
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("Over conveyor", multi_planner);
                stage->setGroup(ns + "/ur_manipulator");
                stage->setGoal(m_predefinedPoses.at(LiftPoseName));

                PickupSerial->insert(std::move(stage));
            }
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("Pickup", interpolation_planner);
                stage->setGroup(ns + "/ur_manipulator");
                stage->setIKFrame(ns + "/gripper_link");
                geometry_msgs::msg::PoseStamped pose;
                pose.pose = boxPose;
                pose.header.frame_id = ns + "/world";
                pose.pose.position.z += 0.2;

                pose.pose.orientation.x = -0.5;
                pose.pose.orientation.y = 0.5;
                pose.pose.orientation.z = -0.5;
                pose.pose.orientation.w = -0.5;

                stage->setGoal(pose);
                PickupSerial->insert(std::move(stage));
            }
            {
                auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Move down", cartesian_planner);
                relativeMove->setGroup(ns + "/ur_manipulator");
                relativeMove->setMinMaxDistance(0.0, 1.0f);
                relativeMove->setIKFrame(ns + "/gripper_link");

                geometry_msgs::msg::Vector3Stamped vec;
                vec.header.frame_id = ns + "/world";
                vec.vector.z = -1;
                relativeMove->setDirection(vec);
                PickupSerial->insert(std::move(relativeMove));
            }

            task.add(std::move(PickupSerial));
        }

        return task;
    }

    mtc::Task MTCController::createTaskDrop(
        const Eigen::Vector3f adress,
        std::string boxname,
        geometry_msgs::msg::Pose& palletPose,
        geometry_msgs::msg::Pose& boxPose,
        std::vector<geometry_msgs::msg::Pose> boxes,
        std::string ns)
    {
        mtc::Task task(ns);
        task.stages()->setName("Drop box");
        task.loadRobotModel(node_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

	     auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
	     sampling_planner->setTimeout(10.);

        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);
	     interpolation_planner->setMaxVelocityScalingFactor(0.5);
	     interpolation_planner->setMaxAccelerationScalingFactor(0.5);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(.005);

	     auto multi_planner = std::make_shared<mtc::solvers::MultiPlanner>();
	     multi_planner->push_back(interpolation_planner);
	     multi_planner->push_back(sampling_planner);

	     auto MoveToDrop = std::make_unique<mtc::SerialContainer>("Move to drop");
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Set scene");

            stage->addObject(Utils::CreateBoxCollision(
                boxname, BoxDimension, Utils::fromMsgPosition(boxPose.position), Utils::fromMsgQuaternion(boxPose.orientation), ns));
            stage->allowCollisions(
                boxname, task.getRobotModel()->getJointModelGroup(ns + "/ur_manipulator")->getLinkModelNamesWithCollisionGeometry(), false);
            int counter = 0;
            std::cerr << boxes.size() << std::endl;
            for (auto box : boxes)
            {
                auto boxWithCounter = std::to_string(counter);
                counter++;
                // namesOfBoxes.push_back(boxWithCounter);
                stage->addObject(Utils::CreateBoxCollision(
                    boxWithCounter, BoxDimension, Utils::fromMsgPosition(box.position), Utils::fromMsgQuaternion(boxPose.orientation), ns));
                stage->allowCollisions(
                    boxWithCounter,
                    task.getRobotModel()->getJointModelGroup(ns + "/ur_manipulator")->getLinkModelNamesWithCollisionGeometry(),
                    false);
            }

            stage->addObject(Utils::CreateBoxCollision(
                "pallet",
                PalletDimensions,
                Utils::fromMsgPosition(palletPose.position),
                Utils::fromMsgQuaternion(palletPose.orientation),
                ns));
            stage->allowCollisions(
                "pallet", task.getRobotModel()->getJointModelGroup(ns + "/ur_manipulator")->getLinkModelNamesWithCollisionGeometry(), true);
            MoveToDrop->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach box");
            stage->attachObject(boxname, ns + "/gripper_link");
            MoveToDrop->insert(std::move(stage));
        }
        {
            auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Move up", cartesian_planner);
            relativeMove->setGroup(ns + "/ur_manipulator");
            relativeMove->setMinMaxDistance(0.0, 1.0f);
            relativeMove->setIKFrame(ns + "/gripper_link");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = ns + "/world";
            vec.vector.z = 1;
            relativeMove->setDirection(vec);
            MoveToDrop->insert(std::move(relativeMove));
        }
        {
            auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location", multi_planner);
            moveToDropLocation->setGroup(ns + "/ur_manipulator");
            moveToDropLocation->setGoal(DropConfig);

            MoveToDrop->insert(std::move(moveToDropLocation));
        }
        {
            auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location exact", multi_planner);
            moveToDropLocation->setGroup(ns + "/ur_manipulator");
            moveToDropLocation->setIKFrame(ns + "/gripper_link");

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = ns + "/world";
            pose.header.stamp = node_->now();

            //! Relative position of the drop location

            pose.pose = Utils::getBoxTargetPose(adress + DropRise * Eigen::Vector3f::UnitZ(), palletPose, BoxDimension);

            moveToDropLocation->setGoal(pose);
            MoveToDrop->setTimeout(10.);

            MoveToDrop->insert(std::move(moveToDropLocation));
        }
        {
            auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Place position", cartesian_planner);
            relativeMove->setGroup(ns + "/ur_manipulator");
            relativeMove->setMinMaxDistance(0.0, 0.5f);
            relativeMove->setIKFrame(ns + "/gripper_link");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = ns + "/world";
            vec.vector.z = -1;
            relativeMove->setDirection(vec);
            MoveToDrop->insert(std::move(relativeMove));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Detach box");
            stage->detachObject(boxname, ns + "/gripper_link");
            MoveToDrop->insert(std::move(stage));
        }

        task.add(std::move(MoveToDrop));

        return task;
    }

    mtc::Task MTCController::createTaskPark(std::string ns)
    {
        mtc::Task task(ns);
        task.stages()->setName("Park");
        task.loadRobotModel(node_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

	     auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
	     sampling_planner->setTimeout(10.);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);
	     interpolation_planner->setMaxVelocityScalingFactor(0.5);
	     interpolation_planner->setMaxAccelerationScalingFactor(0.5);
	     auto multi_planner = std::make_shared<mtc::solvers::MultiPlanner>();
	     multi_planner->push_back(interpolation_planner);
	     multi_planner->push_back(sampling_planner);

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("Park position", multi_planner);
            stage->setGroup(ns + "/ur_manipulator");
            stage->setGoal(ns + "/test_configuration");
            task.add(std::move(stage));
        }

        return task;
    }
} // namespace TaskConstructor
