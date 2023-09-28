#include <chrono>
#include <control_msgs/action/detail/gripper_command__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
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
#include <moveit/task_constructor/task.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vision_msgs/msg/detection3_d_array.hpp>
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

#include "gripper.h"
#include "taskConstructor.h"
#include "ur_moveit_demo_msg/action/mtc.hpp"
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

		m_waitTime = std::chrono::seconds(m_node->get_parameter("wait_time").as_int());
		m_numOfBoxes = m_node->get_parameter("num_of_boxes").as_int();
		m_tolerance = m_node->get_parameter("pose_tolerance").as_double();
	}

private:
	void TimerCallback() {
		if (true || m_visionSystem->IsRobotPresent()) {
			if (!m_isExecuting && m_robotLeft) {
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
					m_robotLeft = false;
					m_isExecuting = true;
					std::thread runner([&]() { execute(); });
					runner.detach();
				}
			}
		} else {
			m_robotName = "";
			m_robotLeft = true;
		}
	}

	std::vector<Eigen::Vector3f> putBoxesInPlaces(std::shared_ptr<TaskConstructor::MTCController> mtc_task_node,
	                                              std::shared_ptr<Gripper::GripperController> gripperController,
	                                              const std::vector<Eigen::Vector3f>& targets) {
		bool previousFailed = false;
		std::vector<Eigen::Vector3f> failedBoxes;
		for (size_t i = 0; i < targets.size(); i++) {
			if (previousFailed) {
				mtc::Task taskPark = mtc_task_node->createTaskPark(m_ns);
				mtc_task_node->doTask(taskPark);
				previousFailed = false;
			}

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
				failedBoxes.push_back(address);
				previousFailed = true;
				continue;
			}

			auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");

			mtc::Task taskGrab = mtc_task_node->createTaskGrab(*myClosestBox, PickedBoxName, m_ns);
			if (!mtc_task_node->doTask(taskGrab)) {
				gripperController->Release();
				failedBoxes.push_back(address);
				previousFailed = true;
				continue;
			}
			gripperController->Grip();
			std::this_thread::sleep_for(50ms);
			auto taskDrop =
			    mtc_task_node->createTaskDrop(address, PickedBoxName, *palletPose, *myClosestBox, allBoxesOnPallet, m_ns);
			if (!mtc_task_node->doTask(taskDrop)) {
				gripperController->Release();
				failedBoxes.push_back(address);
				previousFailed = true;
				continue;
			}
			gripperController->Release();
			std::this_thread::sleep_for(50ms);
		}
		return failedBoxes;
	}

	void execute() {
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface("/" + m_ns);
		planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

		auto mtc_task_node = std::make_shared<TaskConstructor::MTCController>(m_node, m_ns);
		auto gripperController = std::make_shared<Gripper::GripperController>(m_node, "/" + m_ns + "/gripper_server");

		if (m_numOfBoxes > Pattern.size()) {
			RCLCPP_WARN_STREAM(m_node->get_logger(), "Reducing box task to available pattern size");
			m_numOfBoxes = Pattern.size();
		}

		std::vector<Eigen::Vector3f> boxesTargetPlaces;
		boxesTargetPlaces.resize(m_numOfBoxes);
		float scaling = 1.2f;
		std::transform(Pattern.begin(), Pattern.begin() + m_numOfBoxes, boxesTargetPlaces.begin(),
		               [scaling](Eigen::Vector3f element) { return element * scaling; });
		auto failedBoxes = putBoxesInPlaces(mtc_task_node, gripperController, boxesTargetPlaces);

		// Retry once for failed boxes
		failedBoxes = putBoxesInPlaces(mtc_task_node, gripperController, failedBoxes);
		if (failedBoxes.size() > 0) {
			RCLCPP_WARN_STREAM(m_node->get_logger(), "Failed boxes " << failedBoxes.size());
		}

		mtc::Task taskPark = mtc_task_node->createTaskPark(m_ns);
		mtc_task_node->doTask(taskPark);

		auto slashLocation = m_robotName.find("/");
		auto cargoPublisher = m_node->create_publisher<std_msgs::msg::Bool>(
		    "/" + m_robotName.substr(0, slashLocation + 1) + "cargo_status", 1);
		std_msgs::msg::Bool cargoFullMessage;
		cargoFullMessage.data = true;
		cargoPublisher->publish(cargoFullMessage);

		m_isExecuting = false;
	}

	rclcpp::Node::SharedPtr m_node;
	std::shared_ptr<Camera::GroundTruthCamera> m_visionSystem;
	rclcpp::TimerBase::SharedPtr m_timer;
	std::string m_ns;

	Eigen::Vector3d m_lastRobotPose{ 0, 0, 0 };
	std::string m_robotName;
	bool m_isExecuting = false;
	bool m_robotLeft = true;

	std::chrono::time_point<std::chrono::system_clock> lastUpdate{ std::chrono::system_clock::now() };

	double m_tolerance = 0.01;
	std::chrono::seconds m_waitTime{ 3 };
	int m_numOfBoxes = 1;
};

int main(int argc, char** argv) {
	std::mutex locationsMutex;
	std::optional<geometry_msgs::msg::Pose> closestBox;
	std::map<std::string, geometry_msgs::msg::Pose> objectPoses;
	std::vector<geometry_msgs::msg::Pose> allBoxesOnPallet;

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
