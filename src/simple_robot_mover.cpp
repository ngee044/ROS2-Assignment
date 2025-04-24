#include <rclcpp/rclcpp.hpp>
#include <dsr_msgs2/srv/move_joint.hpp>
#include <cmath>

using namespace std::chrono_literals;

class SimpleRobotMover : public rclcpp::Node {
public:
	SimpleRobotMover() : Node("simple_robot_mover"), angle_(0.0) 
	{

		// using the service name "/motion/move_joint" <-- ROS2 Service
		client_ = create_client<dsr_msgs2::srv::MoveJoint>("/motion/move_joint");

		// Create a timer to call the timer_callback function every 10ms
		timer_ = create_wall_timer(10ms, std::bind(&SimpleRobotMover::timer_callback, this));
	}

private:
	void timer_callback() 
	{
		if (!client_->service_is_ready())
		{
			RCLCPP_INFO(get_logger(), "[SimpleRobotMover] Service not ready");
			return;
		}

		auto request = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();
        
		request->pos[0] = 30.0 * sin(angle_);     // J1: Base 회전 [-30~30도]
		request->pos[1] = -20.0;                  // J2: 고정 각도
		request->pos[2] = 90.0;                   // J3: 고정 각도
		request->pos[3] = 0.0;                    // J4: 고정
		request->pos[4] = -60.0;                  // J5: 고정
		request->pos[5] = 45.0 * cos(angle_);     // J6: Wrist 회전 [-45~45도]
		
		request->vel = 30.0;  // 30% 속도
		request->acc = 30.0;  // 30% 가속도
		request->time = 0.0;
		request->mode = 0;
		request->radius = 0.0;

		angle_ += 0.2;  // 각도 증가량 (라디안 아님)
		if(angle_ > 2*M_PI) 
		{
			angle_ -= 2*M_PI;
		}

		// Send the request to the service
		// `async_send_request` is a non-blocking call
        client_->async_send_request(request);
	}

	rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr client_;
	rclcpp::TimerBase::SharedPtr timer_;
	double angle_;
};

int main(int argc, char** argv) 
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimpleRobotMover>());
	rclcpp::shutdown();
	return 0;
}