#include "palletizationNode.h"

PalletizationNode::PalletizationNode(rclcpp::Node::SharedPtr node, const std::string& ns)
    : m_node(node), m_ns(ns)
{
    m_visionSystem = std::make_shared<Camera::GroundTruthCamera>(
        node,
        "/" + ns + "/camera_pickup/detections3D",
        "/" + ns + "/camera_drop/detections3D",
        ns);

    m_timer = m_node->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&PalletizationNode::TimerCallback, this));
    m_statusDescriptionPublisher = m_node->create_publisher<std_msgs::msg::String>("/" + ns + "/status_description",
                                                                                   10);
    m_waitTime = std::chrono::seconds(m_node->get_parameter("wait_time").as_int());
    m_numOfBoxes = m_node->get_parameter("num_of_boxes").as_int();
    m_tolerance = m_node->get_parameter("pose_tolerance").as_double();
    RCLCPP_INFO(m_node->get_logger(), "Creating moeit interface");

    robot_model_loader::RobotModelLoader robot_model_loader(node);
    m_kinematic_model = robot_model_loader.getModel();

    moveit::planning_interface::MoveGroupInterface::Options options(ns + "/ur_manipulator",
                                                                    robot_model_loader.getRobotDescription(),
                                                                    "/" + ns);
    m_moveGroupIterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options);
    m_moveGroupIterface->startStateMonitor();

    const auto pose = m_moveGroupIterface->getCurrentPose();
    RCLCPP_INFO(m_node->get_logger(), "Current pose: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}


void PalletizationNode::SendStatus(const char* format, ...)
{
    assert(m_statusDescriptionPublisher);
    // Create a variable argument list
    va_list args;
    va_start(args, format);

    // Use vsnprintf to format the log message
    const int bufferSize = 256;
    char buffer[bufferSize];
    vsnprintf(buffer, bufferSize, format, args);

    std_msgs::msg::String msg;
    msg.data = buffer;
    m_statusDescriptionPublisher->publish(msg);
}

bool PalletizationNode::CheckIfAmrIsStationary(const geometry_msgs::msg::Pose& robotPose)
{
    const Eigen::Vector3d currentRobotPose{robotPose.position.x, robotPose.position.y, robotPose.position.z};
    const Eigen::Vector3d difference = currentRobotPose - m_lastRobotPose;
    if (difference.norm() > m_tolerance)
    {
      m_lastUpdate = std::chrono::system_clock::now();
    }

    m_lastRobotPose = currentRobotPose;
    return std::chrono::system_clock::now() - m_lastUpdate > m_waitTime;
}

void PalletizationNode::TimerCallback()
{
    if (m_visionSystem->IsRobotPresent())
    {
        if (m_threadPalletization && !m_isExecuting && m_threadPalletization->joinable())
        {
            m_threadPalletization->join();
            m_threadPalletization = nullptr;
            m_robotName = "";
        }

        if (!m_isExecuting && m_threadPalletization == nullptr)
        {
            if (m_robotName == "")
            {
                RCLCPP_INFO(m_node->get_logger(), "Robot detected");
                m_robotName = m_visionSystem->GetRobotName();
            }

            geometry_msgs::msg::Pose robotPose = *m_visionSystem->GetObjectPose(m_robotName);
            const bool amrStopped = CheckIfAmrIsStationary(robotPose);
            if (amrStopped)
            {
                RCLCPP_INFO(m_node->get_logger(), "Starting palletization");
                m_isExecuting.store(true);
                m_threadPalletization = std::make_unique<std::thread>([&]() { Execute(m_robotName); });
            }
        }
    }
}

std::vector<Eigen::Vector3f> PalletizationNode::PutBoxesInPlaces(
    std::shared_ptr<Palletization::RoboticArmController> robotArmController,
    std::shared_ptr<Gripper::GripperController> gripperController,
    const std::vector<Eigen::Vector3f>& targets)
{
    using namespace std::chrono_literals;
    namespace PalCnt = Palletization::Constants;

    std::vector<Eigen::Vector3f> failedBoxes;

    for (size_t i = 0; i < targets.size(); i++)
    {
        SendStatus("Started placing box %d", i);
        const auto& address = targets[i];
        std::optional<geometry_msgs::msg::Pose> myClosestBox = m_visionSystem->GetClosestBox();

        auto palletPose = m_visionSystem->GetObjectPose("/EuroPallet");
        if (!palletPose)
        {
            break;
        }

        if (!myClosestBox)
        {
            RCLCPP_ERROR(m_node->get_logger(), "No box found");
            continue;
        }

        if (!robotArmController->SetPosePIP(Palletization::RoboticArmController::PickupPoseName))
        {
            std::abort();
        }

        // update box position before grabbing.
        bool boxIsMoving = true;
        while (boxIsMoving)
        {
            const auto boxCheck = m_visionSystem->GetClosestBox();
            if (boxCheck)
            {
                const auto diff = Utils::FromMsgPosition(boxCheck->position) -
                            Utils::FromMsgPosition(myClosestBox->position);
                if (diff.norm() < 0.01)
                {
                    boxIsMoving = false;
                }
                else
                {
                    std::this_thread::sleep_for(100ms);
                }
                myClosestBox = boxCheck;
            }
        }

        assert(myClosestBox); // myClosestBox should be set by now
        const auto currentOrientation = robotArmController->GetCurrentOrientation();
        const auto orientationDown = Utils::GetClosestQuaternionFromList(currentOrientation, PalCnt::OrientationsDown);
        SendStatus("Get box from pose: %f %f %f", myClosestBox->position.x, myClosestBox->position.y,
                   myClosestBox->position.z);
        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(myClosestBox->position) + PalCnt::PickupZStartOffset,
                orientationDown))
        {
            std::abort();
        }

        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(myClosestBox->position) + PalCnt::PickupZStopOffset,
                orientationDown,
                0.1))
        {
            std::abort();
        }

        gripperController->Grip();
        std::this_thread::sleep_for(50ms);

        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(myClosestBox->position) + PalCnt::PickupZStartOffset2,
                orientationDown,
                0.25))
        {
            std::abort();
        }

        robotArmController->SetPosePIP(Palletization::RoboticArmController::DropPoseName);

        const auto poseExact = Utils::GetBoxTargetPose(
            address, *palletPose, PalCnt::BoxDimension, PalCnt::Separation);

        SendStatus("Put box in pose: %f %f %f", poseExact.position.x, poseExact.position.y, poseExact.position.z);

        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(poseExact.position) + PalCnt::DropZStartOffset,
                Utils::FromMsgQuaternion(poseExact.orientation)))
        {
            std::abort();
        }

        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(poseExact.position) + PalCnt::PickupZOffset,
                Utils::FromMsgQuaternion(poseExact.orientation),
                0.05))
        {
            std::abort();
        }

        std::this_thread::sleep_for(100ms);
        gripperController->Release();
        std::this_thread::sleep_for(50ms);

        if (!robotArmController->SetPosePIP(
                Utils::FromMsgPosition(poseExact.position) + PalCnt::DropZStartOffset,
                Utils::FromMsgQuaternion(poseExact.orientation)))
        {
            std::abort();
        }

        SendStatus("Done placing box %d", i);
    }
    return failedBoxes;
}

void PalletizationNode::Execute(const std::string &amrName)
{
    using namespace std::chrono_literals;
    namespace PalCnt = Palletization::Constants;

    RCLCPP_INFO(m_node->get_logger(), "Starting task, amr name: %s", amrName.c_str());
    SendStatus("Starting task, amr name: %s", amrName.c_str());
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/" + m_ns);
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

    auto robotArmController = std::make_shared<Palletization::RoboticArmController>(m_moveGroupIterface, m_ns);
    auto gripperController = std::make_shared<Gripper::GripperController>(m_node, "/" + m_ns + "/gripper_server");

    if (m_numOfBoxes > PalCnt::Pattern.size())
    {
        RCLCPP_WARN_STREAM(m_node->get_logger(), "Reducing box task to available pattern size");
        m_numOfBoxes = PalCnt::Pattern.size();
    }

    robotArmController->SetPosePIP(Palletization::RoboticArmController::PickupPoseName);

    std::vector<Eigen::Vector3f> boxesTargetPlaces;
    boxesTargetPlaces.resize(m_numOfBoxes);
    const float scaling = 1.05f;
    std::transform(PalCnt::Pattern.begin(), PalCnt::Pattern.begin() + m_numOfBoxes,
                   boxesTargetPlaces.begin(),
                   [scaling](Eigen::Vector3f element) { return element * scaling; });

    const auto failedBoxes = PutBoxesInPlaces(robotArmController, gripperController,
                                        boxesTargetPlaces);
    SendStatus("Number of failed boxes %d ", failedBoxes.size());
    PutBoxesInPlaces(robotArmController, gripperController, failedBoxes);

    robotArmController->SetPosePIP(Palletization::RoboticArmController::PickupPoseName);

    const auto slashLocation = m_robotName.find("/");
    const auto cargoStatusMsg = "/" + m_robotName.substr(0, slashLocation + 1) + "cargo_status";
    auto cargoPublisher = m_node->create_publisher<std_msgs::msg::Bool>(cargoStatusMsg, 1);
    std_msgs::msg::Bool cargoFullMessage;
    cargoFullMessage.data = true;
    cargoPublisher->publish(cargoFullMessage);

    // wait for robot to leave
    while (m_visionSystem->IsRobotPresent())
    {
        RCLCPP_INFO(m_node->get_logger(), "Wait for robot to go away, resending request at topic %s ", cargoStatusMsg.c_str());
        SendStatus("Wait for robot to go away, resending request at topic %s ", cargoStatusMsg.c_str());
        cargoPublisher->publish(cargoFullMessage);
        std::this_thread::sleep_for(1s);
    }

    RCLCPP_INFO(m_node->get_logger(), "Finished task, amr name: %s", amrName.c_str());
    SendStatus("Finished task, amr name: %s", amrName.c_str());
    m_isExecuting.store(false);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>(
            "ur_palletization", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto parameter = node->get_parameter("ns");
    auto ns = parameter.as_string();

    PalletizationNode pn(node, ns);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
