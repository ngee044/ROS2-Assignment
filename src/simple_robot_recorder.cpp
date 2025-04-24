#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/srv/get_current_pose.hpp"

using namespace std::chrono_literals;

class SimpleRobotRecorder : public rclcpp::Node
{
public:
	SimpleRobotRecorder() : Node("simple_robot_recorder")
	{

		// using the service name "/system/get_current_pose" <-- ROS2 Service
		client_ = create_client<dsr_msgs2::srv::GetCurrentPose>("/system/get_current_pose");

		// Create a timer to call the timer_callback function every 10ms
		timer_ = create_wall_timer(10ms, 
			std::bind(&SimpleRobotRecorder::timer_callback, this));
	}
private:
	void timer_callback()
	{
		if (!client_->service_is_ready())
		{
			RCLCPP_INFO(get_logger(), "[SimpleRobotRecorder] Service not ready");
			return;
		}

		auto request = std::make_shared<dsr_msgs2::srv::GetCurrentPose::Request>();
		request->space_type = 1;

		client_->async_send_request(request,
			[this](rclcpp::Client<dsr_msgs2::srv::GetCurrentPose>::SharedFuture future)
			{
				// `future` is a shared pointer to the result of the service call
				auto result = future.get();
				if (result)
				{
					RCLCPP_INFO(get_logger(), "TCP : %f %f %f %f %f %f",
						result->pos[0], result->pos[1], result->pos[2],
						result->pos[3], result->pos[4], result->pos[5]);
				}
				else
				{
					RCLCPP_ERROR(get_logger(), "[SimpleRobotRecorder] Failed to get current pose");
				}
			});

	}

	rclcpp::Client<dsr_msgs2::srv::GetCurrentPose>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv)
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<SimpleRobotRecorder>());

	rclcpp::shutdown();
	return 0;
}