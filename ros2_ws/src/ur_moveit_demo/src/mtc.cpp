#include <chrono>
#include <control_msgs/action/detail/gripper_command__struct.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <utility>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

bool SendGripperGrip(rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_ptr);

bool SendGripperrelease(rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_ptr);

constexpr float BoxHeight = 0.3f;
const Eigen::Vector3d TableDimension{ 0.950, 0.950, 0.411 };
const Eigen::Vector3d ConveyorDimensions{ 2.0, 0.7, 0.15 };
const Eigen::Vector3d PickupLocation{ 0.890, 0, 0.049 };
const Eigen::Vector3d BoxDimension{ 0.2, 0.2, 0.2 };

const std::map<std::string, double> PickupConfig{
    { "wrist_1_joint", -0.8385483622550964 },      { "wrist_2_joint", 1.5643877983093262 },       { "elbow_joint", -1.550349235534668 },
    { "shoulder_pan_joint", -2.7139534950256348 }, { "shoulder_lift_joint", -2.314471483230591 }, { "wrist_3_joint", -5.752989768981934 }
};

const std::map<std::string, double> LiftConfig{
    { "wrist_1_joint", -1.6993759870529175 },     { "wrist_2_joint", 1.5634684562683105 },        { "elbow_joint", -1.0284128189086914 },
    { "shoulder_pan_joint", -2.644500255584717 }, { "shoulder_lift_joint", -1.9712634086608887 }, { "wrist_3_joint", -5.683835983276367 }
};
const std::map<std::string, double> DropConfig{
    { "wrist_1_joint", -1.5789473056793213 },       { "wrist_2_joint", 1.5672444105148315 },       { "elbow_joint", -0.9531064033508301 },
    { "shoulder_pan_joint", -0.15679530799388885 }, { "shoulder_lift_joint", -2.199610710144043 }, { "wrist_3_joint", -3.1959822177886963 }
};

const geometry_msgs::msg::Point toMsgPoint(Eigen::Vector3d v)
{
    geometry_msgs::msg::Point msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

const geometry_msgs::msg::Quaternion toMsgQuaternion(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

moveit_msgs::msg::CollisionObject CreateBoxCollision(
    const std::string& name,
    const Eigen::Vector3d dimension,
    const Eigen::Vector3d location,
    const Eigen::Quaterniond& rot = Eigen::Quaterniond::Identity())
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = name;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.push_back(dimension.x());
    primitive.dimensions.push_back(dimension.y());
    primitive.dimensions.push_back(dimension.z());
    collision_object.primitives.push_back(primitive);
    geometry_msgs::msg::Pose box_pose;
    box_pose.position = toMsgPoint(location);
    box_pose.orientation = toMsgQuaternion(rot);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

    void doTask(mtc::Task& task);

    void setupPlanningScene();

    mtc::Task createTaskGrab();

    mtc::Task createTaskDrop(int xOffset, int yOffset);

private:
    // Compose an MTC task from a series of stages.
    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
    return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(
        CreateBoxCollision("table", TableDimension, Eigen::Vector3d{ 0, 0, -TableDimension.z() / 2.0 }));
    planning_scene_interface.applyCollisionObject(
        CreateBoxCollision("conveyor", ConveyorDimensions, Eigen::Vector3d{ ConveyorDimensions.x() / 2 + 0.75, 0, -0.2 }));
    // planning_scene_interface.applyCollisionObject(
    //     CreateBoxCollision("box", BoxDimension, Eigen::Vector3d{ 0.94, 0.25, 0.25 }));

}

void MTCTaskNode::doTask(mtc::Task& task)
{
    task_ = std::move(task);
    try
    {
        task_.init();
    } catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    std::cerr << "after execution" << std::endl;
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return;
}

mtc::Task MTCTaskNode::createTaskGrab()
{
    mtc::Task task;
    task.stages()->setName("Grab box");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "base_link";
    const auto& hand_group_name = "ur_manipulator";
    const auto& hand_frame = "base_link";

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setTimeout(10);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScaling(1.0);
    cartesian_planner->setMaxAccelerationScaling(1.0);
    cartesian_planner->setStepSize(.01);

    {
        auto Pickup = std::make_unique<mtc::SerialContainer>("Pickup");
        {
            auto overConveyor = std::make_unique<mtc::stages::MoveTo>("overConveyor", sampling_planner);
            overConveyor->setGroup("ur_manipulator");
            overConveyor->setGoal(LiftConfig);

            Pickup->insert(std::move(overConveyor));
        }
        {
            auto pickUp = std::make_unique<mtc::stages::MoveTo>("pickup", sampling_planner);
            pickUp->setGroup("ur_manipulator");
            pickUp->setGoal(PickupConfig);  
            Pickup->insert(std::move(pickUp));
            // pickUp->addSolutionCallback(SolutionCallback && cb)
        }

        // {
        //     auto overConveyor = std::make_unique<mtc::stages::MoveTo>("overConveyor", sampling_planner);
        //     overConveyor->setGroup("ur_manipulator");
        //     overConveyor->setGoal(LiftConfig);

        //     Pickup->insert(std::move(overConveyor));
        // }

        task.add(std::move(Pickup));
    }
    // {
    //     auto MoveToDrop = std::make_unique<mtc::SerialContainer>("Move to drop");
    //     {
    //         auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location", sampling_planner);
    //         moveToDropLocation->setGroup("ur_manipulator");
    //         moveToDropLocation->setGoal(DropConfig);

    //         MoveToDrop->insert(std::move(moveToDropLocation));
    //     }
    //     task.add(std::move(MoveToDrop));
    // }

    // auto pickUp =
    //     std::make_unique<mtc::stages::MoveTo>("ur_manipulator pickup", sampling_planner);
    // overConveyor->setGroup("ur_manipulator");
    // overConveyor->setGoal(PickupConfig);
    // task.add(std::move(overConveyor));

    // auto lift =
    //     std::make_unique<mtc::stages::MoveTo>("ur_manipulator lift", sampling_planner);
    // lift->setGroup("ur_manipulator");
    // lift->setGoal(LiftConfig);
    // task.add(std::move(lift));

    return task;
}

mtc::Task MTCTaskNode::createTaskDrop(int x, int y)
{
    mtc::Task task;
    task.stages()->setName("Drop box");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "base_link";
    const auto& hand_group_name = "ur_manipulator";
    const auto& hand_frame = "base_link";

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
        auto overConveyor = std::make_unique<mtc::stages::MoveTo>("overConveyor", sampling_planner);
        overConveyor->setGroup("ur_manipulator");
        overConveyor->setGoal(LiftConfig);

        MoveToDrop->insert(std::move(overConveyor));
    }
    {
        auto moveToDropLocation = std::make_unique<mtc::stages::MoveTo>("Drop location", interpolation_planner);
        moveToDropLocation->setGroup("ur_manipulator");
        moveToDropLocation->setGoal(DropConfig);

        MoveToDrop->insert(std::move(moveToDropLocation));
    }
    {
        auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Relative Forward", cartesian_planner);
        relativeMove->setGroup("ur_manipulator");
        relativeMove->setMinMaxDistance(0.0, 0.1);
        relativeMove->setIKFrame("gripper_link");

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "gripper_link";
        vec.vector.z = -1;
        relativeMove->setDirection(vec);
        MoveToDrop->insert(std::move(relativeMove));
    }
    if (x != 0)
    {
        auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Relative X", cartesian_planner);
        relativeMove->setGroup("ur_manipulator");
        relativeMove->setMinMaxDistance(0.0, x * 0.3);
        relativeMove->setIKFrame("gripper_link");

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "gripper_link";
        vec.vector.z = 1;
        relativeMove->setDirection(vec);
        MoveToDrop->insert(std::move(relativeMove));
    }
    if (y != 0 ) 
    {
        auto relativeMove = std::make_unique<mtc::stages::MoveRelative>("Relative Y", cartesian_planner);
        relativeMove->setGroup("ur_manipulator");
        relativeMove->setMinMaxDistance(0.0, y * 0.3);
        relativeMove->setIKFrame("gripper_link");

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "gripper_link";
        vec.vector.y = 1;
        relativeMove->setDirection(vec);
        MoveToDrop->insert(std::move(relativeMove));
    
    }
    // {
    //     auto stage = std::make_unique<mtc::stages::GeneratePose>("create fake box");
    //     // stage->addObject(const moveit_msgs::msg::CollisionObject &collision_object)
    // }

    task.add(std::move(MoveToDrop));

    // auto pickUp =
    //     std::make_unique<mtc::stages::MoveTo>("ur_manipulator pickup", sampling_planner);
    // overConveyor->setGroup("ur_manipulator");
    // overConveyor->setGoal(PickupConfig);
    // task.add(std::move(overConveyor));

    // auto lift =
    //     std::make_unique<mtc::stages::MoveTo>("ur_manipulator lift", sampling_planner);
    // lift->setGroup("ur_manipulator");
    // lift->setGoal(LiftConfig);
    // task.add(std::move(lift));

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor_mtc;

    auto const node =
        std::make_shared<rclcpp::Node>("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread(
        [&executor]()
        {
            executor.spin();
        });

    auto spin_thread = std::make_unique<std::thread>(
        [&executor_mtc, &mtc_task_node]()
        {
            executor_mtc.add_node(mtc_task_node->getNodeBaseInterface());
            executor_mtc.spin();
            executor_mtc.remove_node(mtc_task_node->getNodeBaseInterface());
        });

        // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
    };
    moveit_visual_tools.loadRemoteControl();
    moveit_visual_tools.trigger();

    auto client_ptr = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node, "/gripper_server");

    mtc_task_node->setupPlanningScene();
    // mtc::Task taskGrab = mtc_task_node->createTaskGrab();
    // mtc::Task& lastDrop = taskGrab;

        moveit_visual_tools.prompt("  ");
    for (int i = 0; i < 6; i++) {
        mtc::Task taskGrab = mtc_task_node->createTaskGrab();
        auto taskDrop = mtc_task_node->createTaskDrop(i / 3, i % 3);
        mtc_task_node->doTask(taskGrab);
        // moveit_visual_tools.prompt("  ");
        SendGripperGrip(client_ptr);
        // moveit_visual_tools.prompt("  ");
        mtc_task_node->doTask(taskDrop);
        // moveit_visual_tools.prompt("  ");
        SendGripperrelease(client_ptr);
        // lastDrop = std::move(taskDrop);
    }

    // auto const node =
    //     std::make_shared<rclcpp::Node>("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // // // Create a ROS logger
    // // auto const logger = rclcpp::get_logger("ur_moveit_demo");

    // // // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // auto spinner = std::thread(
    //     [&executor]()
    //     {
    //         executor.spin();
    //     });

    // robot_model_loader::RobotModelLoader robot_model_loader(node);
    // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    // RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

    // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    // robot_state->setToDefaultValues();
    // const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");

    // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // for (auto joint_name : joint_names)
    // {
    //     RCLCPP_INFO(LOGGER, "Joint %s", joint_name.c_str());
    // }

    // // Create the MoveIt MoveGroup Interface
    // using moveit::planning_interface::MoveGroupInterface;
    // auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    // auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    //     node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
    // };
    // moveit_visual_tools.loadRemoteControl();
    // moveit_visual_tools.trigger();

    rclcpp::shutdown();
    spin_thread->join();
    spinner.join();
    return 0;
}

bool Grip(rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_ptr, bool grip)
{
    auto goal = control_msgs::action::GripperCommand::Goal();
    goal.command.position = grip ? 0.0 : 1.0;
    goal.command.max_effort = 10000.0;
    auto future = client_ptr->async_send_goal(goal);

    future.wait();

    auto goal_handle = future.get();

    auto result_future = client_ptr->async_get_result(goal_handle);

    result_future.wait();

    auto result = result_future.get().result;

    return result->reached_goal;
}

bool SendGripperGrip(rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_ptr)
{
    return Grip(client_ptr, true);
}

bool SendGripperrelease(rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_ptr)
{
    return Grip(client_ptr, false);
}