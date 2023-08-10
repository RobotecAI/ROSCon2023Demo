#include "taskConstructor.h"
#include "utils.h"

namespace TaskConstructor
{

    MTCController::MTCController(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
    }

    void MTCController::doTask(mtc::Task& task)
    {
        task_ = std::move(task);
        try
        {
            task_.init();
        } catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(LOGGER, e);
            std::abort();
            return;
        }

        if (!task_.plan(5))
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            std::abort();
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            std::abort();
            return;
        }

        return;
    }

    mtc::Task MTCController::createTaskGrab(const geometry_msgs::msg::Pose& boxPose, std::string boxname)
    {
        // ToDo make use of boxPose parameter
        mtc::Task task;
        task.stages()->setName("Grab box");
        task.loadRobotModel(node_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        sampling_planner->setTimeout(10.);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);

        // auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        // cartesian_planner->setMaxVelocityScaling(1.0);
        // cartesian_planner->setMaxAccelerationScaling(1.0);
        // cartesian_planner->setStepSize(.01);

        {
            auto PickupSerial = std::make_unique<mtc::SerialContainer>("Pickup");
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Set scene");
                stage->addObject(Utils::CreateBoxCollision(boxname, BoxDimension, Utils::fromMsgPosition(boxPose.position)));
                stage->allowCollisions(
                    boxname, task.getRobotModel()->getJointModelGroup("ur_manipulator")->getLinkModelNamesWithCollisionGeometry(), false);
                PickupSerial->insert(std::move(stage));
            }
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("Over conveyor", sampling_planner);
                stage->setGroup("ur_manipulator");
                stage->setGoal(LiftConfig);

                PickupSerial->insert(std::move(stage));
            }
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("Pickup", interpolation_planner);
                stage->setGroup("ur_manipulator");
                stage->setIKFrame("gripper_link");
                geometry_msgs::msg::PoseStamped pose;
                pose.pose = boxPose;
                pose.header.frame_id = "world";
                pose.pose.position.z += 0.2;

                pose.pose.orientation.x = -0.5;
                pose.pose.orientation.y = 0.5;
                pose.pose.orientation.z = -0.5;
                pose.pose.orientation.w = -0.5;

                stage->setGoal(pose);
                PickupSerial->insert(std::move(stage));
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
        std::vector<geometry_msgs::msg::Pose> boxes)
    {
        mtc::Task task;
        task.stages()->setName("Drop box");
        task.loadRobotModel(node_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScaling(1.0);
        cartesian_planner->setMaxAccelerationScaling(1.0);
        cartesian_planner->setStepSize(.01);

        auto MoveToDrop = std::make_unique<mtc::SerialContainer>("Move to drop");
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Set scene");

            stage->addObject(Utils::CreateBoxCollision(boxname, BoxDimension, Utils::fromMsgPosition(boxPose.position), Utils::fromMsgQuaternion(boxPose.orientation)));
            stage->allowCollisions(
                boxname, task.getRobotModel()->getJointModelGroup("ur_manipulator")->getLinkModelNamesWithCollisionGeometry(), false);
            int counter = 0;
            for (auto box : boxes)
            {
                auto boxWithCounter = std::to_string(counter);
                counter++;
                // namesOfBoxes.push_back(boxWithCounter);
                stage->addObject(Utils::CreateBoxCollision(boxWithCounter, BoxDimension, Utils::fromMsgPosition(box.position), Utils::fromMsgQuaternion(boxPose.orientation)));
                stage->allowCollisions(
                    boxWithCounter,
                    task.getRobotModel()->getJointModelGroup("ur_manipulator")->getLinkModelNamesWithCollisionGeometry(),
                    false);
            }

            stage->addObject(Utils::CreateBoxCollision(
                "pallet", PalletDimensions, Utils::fromMsgPosition(palletPose.position), Utils::fromMsgQuaternion(palletPose.orientation)));
            stage->allowCollisions(
                "pallet", task.getRobotModel()->getJointModelGroup("ur_manipulator")->getLinkModelNamesWithCollisionGeometry(), true);
            MoveToDrop->insert(std::move(stage));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach box");
            stage->attachObject(boxname, "gripper_link");
            MoveToDrop->insert(std::move(stage));
        }
        {
            auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Move up", cartesian_planner);
            relativeMove->setGroup("ur_manipulator");
            relativeMove->setMinMaxDistance(0.0, 1.0f);
            relativeMove->setIKFrame("gripper_link");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1;
            relativeMove->setDirection(vec);
            MoveToDrop->insert(std::move(relativeMove));
        }
        {
            auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location", interpolation_planner);
            moveToDropLocation->setGroup("ur_manipulator");
            moveToDropLocation->setGoal(DropConfig);

            MoveToDrop->insert(std::move(moveToDropLocation));
        }
        {
            auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location exact", interpolation_planner);
            moveToDropLocation->setGroup("ur_manipulator");
            moveToDropLocation->setIKFrame("gripper_link");

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = node_->now();

            //! Relative position of the drop location
            constexpr float DropRise = 1.8f;
            pose.pose = Utils::getBoxTargetPose(adress + DropRise * Eigen::Vector3f::UnitZ(), palletPose, BoxDimension);

            moveToDropLocation->setGoal(pose);
            MoveToDrop->setTimeout(10.);

            MoveToDrop->insert(std::move(moveToDropLocation));
        }
        {
            auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Place position", cartesian_planner);
            relativeMove->setGroup("ur_manipulator");
            relativeMove->setMinMaxDistance(0.0, 0.5f);
            relativeMove->setIKFrame("gripper_link");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -1;
            relativeMove->setDirection(vec);
            MoveToDrop->insert(std::move(relativeMove));
        }
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Detach box");
            stage->detachObject(boxname, "gripper_link");
            MoveToDrop->insert(std::move(stage));
        }

        task.add(std::move(MoveToDrop));

        return task;
    }

    mtc::Task MTCController::createTaskPark() {
        mtc::Task task;
        task.stages()->setName("Park");
        task.loadRobotModel(node_);

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(stage_state_current));

        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        interpolation_planner->setTimeout(10);

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("Park position", interpolation_planner);
            stage->setGroup("ur_manipulator");
            stage->setGoal("test_configuration");
            task.add(std::move(stage));
        }

        return task;
    }
} // namespace TaskConstructor
