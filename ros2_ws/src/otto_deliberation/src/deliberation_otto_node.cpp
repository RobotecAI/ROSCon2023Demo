
#include <otto_deliberation/otto_autonomy.h>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

class OttoNode : public rclcpp::Node
{
public:
	OttoNode() : Node("otto_deliberation_node"), m_autonomy(shared_from_this()) {
		RCLCPP_INFO(get_logger(), "Otto deliberation node starting");
		m_namespace = get_namespace();

		m_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OttoNode::TimerCallback, this));
	}

private:
	void TimerCallback() {  // Get tasks, updates etc. here
		m_autonomy.Update();
	}

	// TODO:
	// Listen to routine (task list), set it for otto autonomy
	// Listen to events of loading / unloading for your lane, ignore if not in correct status (e.g. WAIT_LOAD).

	rclcpp::TimerBase::SharedPtr m_timer;
	std::string m_namespace;
	OttoAutonomy m_autonomy;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto otto_node = std::make_shared<OttoNode>();
	rclcpp::spin(otto_node);
	rclcpp::shutdown();
	RCLCPP_INFO(otto_node->get_logger(), "Otto terminating");
	return 0;
}
