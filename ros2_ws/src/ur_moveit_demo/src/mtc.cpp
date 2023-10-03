#include <chrono>

#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <utility>

#include <vision_msgs/msg/detection3_d_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include "gripper.h"
#include "taskConstructor.h"
#include "utils.h"
#include "vision.h"

constexpr char BoxNamePrefix[] = "Box";
constexpr char PalletNamePrefix[] = "EuroPallet";
constexpr char PickedBoxName[] = "PickedBox";

const std::vector<Eigen::Vector3f> Pattern{
	//    { -1.f, 0.5, 1.f }, { 0.f, 0.5f, 1.f }, { 1.f, 0.5f, 1.f },{ -1.f, -0.5f, 1.f }, { 0.f, -0.5f, 1.f }, { 1.f,
	//    -0.5f, 1.f },
	//    { -1.f, 0.5, 2.f }, { 0.f, 0.5f, 2.f }, { 1.f, 0.5f, 2.f }, { -1.f, -0.5f, 2.f }, { 0.f, -0.5f, 2.f }, { 1.f,
	//    -0.5f, 2.f },
	{ -1.0f, 0.5f, 1.f },  { 0.0f, 0.5f, 1.f },  { 1.0f, 0.5f, 1.f },  { -1.0f, -0.5f, 1.f }, { 0.0f, -0.5f, 1.f },
	{ 1.0f, -0.5f, 1.f },  { -1.0f, 0.5f, 2.f }, { 0.0f, 0.5f, 2.f },  { 1.0f, 0.5f, 2.f },   { -1.0f, -0.5f, 2.f },
	{ 0.0f, -0.5f, 2.f },  { 1.0f, -0.5f, 2.f }, { -1.0f, 0.5f, 3.f }, { 0.0f, 0.5f, 3.f },   { 1.0f, 0.5f, 3.f },
	{ -1.0f, -0.5f, 3.f }, { 0.0f, -0.5f, 3.f }, { 1.0f, -0.5f, 3.f },

};

using namespace std::chrono_literals;

class RobotDetection
{
public:
	RobotDetection(rclcpp::Node::SharedPtr node, std::string ns) : m_node(node), m_ns(ns) {
		m_visionSystem = std::make_shared<Camera::GroundTruthCamera>(node, "/" + ns + "/camera_pickup/detections3D",
		                                                             "/" + ns + "/camera_drop/detections3D", ns);

		m_timer =
		    m_node->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RobotDetection::TimerCallback, this));
        m_statusDescriptionPublisher = m_node->create_publisher<std_msgs::msg::String>("/" + ns + "/status_description", 10);
		m_waitTime = std::chrono::seconds(m_node->get_parameter("wait_time").as_int());
		m_numOfBoxes = m_node->get_parameter("num_of_boxes").as_int();
		m_tolerance = m_node->get_parameter("pose_tolerance").as_double();
        RCLCPP_INFO(m_node->get_logger(), "Creating moeit interface");
        robot_model_loader::RobotModelLoader robot_model_loader(node);
        m_kinematic_model = robot_model_loader.getModel();
        moveit::planning_interface::MoveGroupInterface::Options options(ns + "/ur_manipulator", robot_model_loader.getRobotDescription(), "/" + ns);
        m_move_groupIterface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options);
        m_move_groupIterface->startStateMonitor();

        auto pose = m_move_groupIterface->getCurrentPose();
        RCLCPP_INFO(m_node->get_logger(), "Current pose: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    }


private:

    void SendStatus(const char* format, ...)
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

	void TimerCallback() {
		if (m_visionSystem->IsRobotPresent()) {

            if (m_threadPalletization && !m_isExecuting && m_threadPalletization->joinable())
            {
                m_threadPalletization->join();
                m_threadPalletization = nullptr;
            }
			if (!m_isExecuting && m_threadPalletization == nullptr) {
				if (m_robotName == "") {
					RCLCPP_INFO(m_node->get_logger(), "Robot detected");
					m_robotName = m_visionSystem->GetRobotName();
				}
				geometry_msgs::msg::Pose robotPose = *m_visionSystem->getObjectPose(m_robotName);
				Eigen::Vector3d currentRobotPose{ robotPose.position.x, robotPose.position.y, robotPose.position.z };
				Eigen::Vector3d difference = currentRobotPose - m_lastRobotPose;
				if (difference.norm() > m_tolerance) {
					lastUpdate = std::chrono::system_clock::now();
				}
				m_lastRobotPose = currentRobotPose;
				if (std::chrono::system_clock::now() - lastUpdate > m_waitTime) {
					RCLCPP_INFO(m_node->get_logger(), "Starting palletization");
					m_isExecuting.store(true);
                    m_threadPalletization = std::make_unique<std::thread>([&]() { execute(m_robotName); });
				}
			}
		}
	}


	std::vector<Eigen::Vector3f> putBoxesInPlaces(std::shared_ptr<TaskConstructor::MTCController> mtc_task_node,
	                                              std::shared_ptr<Gripper::GripperController> gripperController,
	                                              const std::vector<Eigen::Vector3f>& targets) {
		bool previousFailed = false;
		std::vector<Eigen::Vector3f> failedBoxes;
		for (size_t i = 0; i < targets.size(); i++) {
            SendStatus("Started placing box %d", i);
			auto const& address = targets[i];
			std::optional<geometry_msgs::msg::Pose> myClosestBox;
			std::vector<geometry_msgs::msg::Pose> allBoxesOnPallet;

			myClosestBox = m_visionSystem->getClosestBox();
			allBoxesOnPallet = m_visionSystem->getAllBoxesOnPallet();

			auto palletPose = m_visionSystem->getObjectPose("/EuroPallet");
			if (!palletPose) {
				break;
			}

			if (!myClosestBox) {
				RCLCPP_ERROR(m_node->get_logger(), "No box found");

				previousFailed = true;
				continue;
			}

			auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");


			if (!mtc_task_node->setPosePIP(TaskConstructor::MTCController::PickupPoseName))
			{
				std::abort();
			}
			// update box position before grabbing.
			bool boxIsMoving = true;
			while (boxIsMoving)
			{
				const auto boxCheck = m_visionSystem->getClosestBox();
				if (boxCheck)
				{
					auto diff = Utils::fromMsgPosition(boxCheck->position) - Utils::fromMsgPosition(myClosestBox->position);
					if (diff.norm() < 0.01)
					{
						boxIsMoving = false;
					}
					else{
						std::this_thread::sleep_for(100ms);
					}
					myClosestBox = boxCheck;
				}
			}

            assert(myClosestBox); // myClosestBox should be set by now
            auto pose = m_move_groupIterface->getCurrentPose();
			auto currentOrientation = mtc_task_node->getCurrentOrientation();
			auto orientationDown = Utils::GetClosestQuaternionFromList(currentOrientation, OrientationsDown);
            SendStatus("Get box from pose: %f %f %f", myClosestBox->position.x, myClosestBox->position.y, myClosestBox->position.z);
			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(myClosestBox->position)+ PickupZStartOffset, orientationDown))
			{
				std::abort();
			}

			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(myClosestBox->position) + PickupZStopOffset, orientationDown, 0.1, "LIN"))
			{
				std::abort();
			}

			gripperController->Grip();
			std::this_thread::sleep_for(50ms);
			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(myClosestBox->position) + PickupZStartOffset2, orientationDown, 0.25, "LIN" ))
			{
				std::abort();
			}

            mtc_task_node->setPosePIP(TaskConstructor::MTCController::DropPoseName);

			auto currentOrientationDrop = mtc_task_node->getCurrentOrientation();
			auto orientationDownDrop = Utils::GetClosestQuaternionFromList(currentOrientationDrop, OrientationsDown);

			auto poseExact = Utils::getBoxTargetPose(address, *palletPose, BoxDimension, Separation);

            SendStatus("Put box in pose: %f %f %f", poseExact.position.x, poseExact.position.y, poseExact.position.z);

			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(poseExact.position) + DropZStartOffset, Utils::fromMsgQuaternion(poseExact.orientation)))
			{
				std::abort();
			}
			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(poseExact.position) + PickupZOffset, Utils::fromMsgQuaternion(poseExact.orientation), 0.05, "LIN"))
			{
				std::abort();
			}
			std::this_thread::sleep_for(100ms);

			gripperController->Release();
			std::this_thread::sleep_for(50ms);
			if (!mtc_task_node->setPosePIP(Utils::fromMsgPosition(poseExact.position) + DropZStartOffset, Utils::fromMsgQuaternion(poseExact.orientation)))
			{
				std::abort();
			}
            SendStatus("Done placing box %d", i);
		}
		return failedBoxes;
	}

	void execute(const std::string& amrName ) {

        RCLCPP_INFO(m_node->get_logger(), "Starting task, amr name: %s", amrName.c_str());
        SendStatus("Starting task, amr name: %s", amrName.c_str());
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/" + m_ns);
		planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

		auto mtc_task_node = std::make_shared<TaskConstructor::MTCController>(m_move_groupIterface, m_ns);
		auto gripperController = std::make_shared<Gripper::GripperController>(m_node, "/" + m_ns + "/gripper_server");

		if (m_numOfBoxes > Pattern.size()) {
			RCLCPP_WARN_STREAM(m_node->get_logger(), "Reducing box task to available pattern size");
			m_numOfBoxes = Pattern.size();
		}

		mtc_task_node->setPosePIP(TaskConstructor::MTCController::PickupPoseName);

		std::vector<Eigen::Vector3f> boxesTargetPlaces;
		boxesTargetPlaces.resize(m_numOfBoxes);
		float scaling = 1.05f;
		std::transform(Pattern.begin(), Pattern.begin() + m_numOfBoxes, boxesTargetPlaces.begin(),
		               [scaling](Eigen::Vector3f element) { return element * scaling; });

		auto failedBoxes = putBoxesInPlaces(mtc_task_node, gripperController, boxesTargetPlaces);
        SendStatus( "Number of failed boxes %d ", failedBoxes.size());
        putBoxesInPlaces(mtc_task_node, gripperController, failedBoxes);

		mtc_task_node->setPosePIP(TaskConstructor::MTCController::PickupPoseName);

		auto slashLocation = m_robotName.find("/");
        const auto cargoStatusMsg = "/" + m_robotName.substr(0, slashLocation + 1) + "cargo_status";
		auto cargoPublisher = m_node->create_publisher<std_msgs::msg::Bool>(cargoStatusMsg, 1);
		std_msgs::msg::Bool cargoFullMessage;
		cargoFullMessage.data = true;
		cargoPublisher->publish(cargoFullMessage);

        // wait for robot to left
        while(m_visionSystem->IsRobotPresent())
        {
            RCLCPP_INFO(m_node->get_logger(), "Wait for robot to go away, resending request at topic %s ", cargoStatusMsg.c_str());
            SendStatus( "Wait for robot to go away, resending request at topic %s ", cargoStatusMsg.c_str());
            cargoPublisher->publish(cargoFullMessage);
            std::this_thread::sleep_for(1s);
        }

        RCLCPP_INFO(m_node->get_logger(), "Finished task, amr name: %s", amrName.c_str());
        SendStatus( "Finished task, amr name: %s", amrName.c_str());
        m_isExecuting.store(false);
	}

	rclcpp::Node::SharedPtr m_node;
	std::shared_ptr<Camera::GroundTruthCamera> m_visionSystem; //Wrapper for vision system
    std::chrono::seconds m_waitTime{ 3 }; //!< Initial wait time for AMR to enter the scene
    std::unique_ptr<std::thread> m_threadPalletization; //!< Thread for palletization task
    moveit::core::RobotModelPtr m_kinematic_model; //!< Robotic arm model
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_groupIterface; //!< Move group interface
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> m_statusDescriptionPublisher; //!< Publisher for cargo status

    rclcpp::TimerBase::SharedPtr m_timer;
	const std::string m_ns; //!< Namespace of the node

	Eigen::Vector3d m_lastRobotPose{ 0, 0, 0 };
	std::string m_robotName;
	std::atomic<bool> m_isExecuting{false}; //!< Flag indicating if task is executing
	std::chrono::time_point<std::chrono::system_clock> lastUpdate{ std::chrono::system_clock::now() };

	double m_tolerance = 0.01; //!< Tolerance for checking if AMR moved

	int m_numOfBoxes = 1;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);

	rclcpp::executors::MultiThreadedExecutor executor_mtc;

	auto const node = std::make_shared<rclcpp::Node>(
	    "mtc_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	auto spinner = std::thread([&executor]() { executor.spin(); });

	auto parameter = node->get_parameter("ns");
	auto ns = parameter.as_string();

	RobotDetection robotDetection(node, ns);

	spinner.join();
	rclcpp::shutdown();
	return 0;
}
