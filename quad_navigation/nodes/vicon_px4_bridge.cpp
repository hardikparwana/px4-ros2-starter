#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <vicon_receiver/msg/position.hpp>
#include "tf2/utils.h"
#include <math.h> 

#define PI 3.14159265

using namespace std::chrono_literals;
using std::placeholders::_1;

class vicon_px4_bridge: public rclcpp::Node
{

	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr publisher_;
	rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr subscription_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	std::atomic<uint64_t> timestamp_;
	px4_msgs::msg::VehicleVisualOdometry pose;

	public: 
		vicon_px4_bridge() : Node("vicon_px4_bridge"), count_(0)
		{
			// Parameters
			this->declare_parameter<float>("publish_freq", 30);
			this->get_parameter("publish_freq",publish_freq);

			// Subscribers and Publishers
			subscription_ = this->create_subscription<vicon_receiver::msg::Position>("/vicon/quad1/quad1",10, std::bind(&vicon_px4_bridge::vicon_callback, this, _1));
			publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in",10);
			timer_ = this->create_wall_timer(500ms, std::bind(&vicon_px4_bridge::px4_callback, this));
			timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out", 10,
								[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
									timestamp_.store(msg->timestamp);
								});

		}

	private:
		
		// Data variables
		
		tf2::Quaternion quat;
		float publish_freq;

		std::string px4_name, vicon_name;

		std::vector<float> position = {0.0, 0.0, 0.0};
		float yaw;
		

		void px4_callback()
		{
			// RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			
			publisher_->publish(pose);
		}

		void vicon_callback(const vicon_receiver::msg::Position::SharedPtr msg)
		{
			pose.timestamp = timestamp_.load();
			pose.timestamp_sample = timestamp_.load();

			pose.x = msg->y_trans/1000;
			pose.y = msg->x_trans/1000;
			pose.z = -msg->z_trans/1000;

			tf2::Quaternion quat(msg->x_rot, msg->y_rot, msg->z_rot,msg->w);
			yaw = -tf2::getYaw(quat) + PI/2.0f;
			pose.q[0] = cos( yaw/2.0f );
			pose.q[3] = sin( yaw/2.0f );

			std::cout << "quat: " << quat.w() << "\t" << quat.x() << "\t" << quat.y() << "\t" << quat.z()  << " yaw: " << -tf2::getYaw(quat) << std::endl;

			// Variables			
			pose.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
			pose.q[1] = 0.0f;
			pose.q[2] = 0.0f;
			pose.q_offset[0] = 1.0;
			pose.q_offset[1] = 0.0;
			pose.q_offset[2] = 0.0;
			pose.q_offset[3] = 0.0;

			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_X_VARIANCE] = 0.01;
			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_Y_VARIANCE] = 0.01;
			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_Z_VARIANCE] = 0.01;
			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_ROLL_VARIANCE] = 0.0523599;
			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_PITCH_VARIANCE] = 0.0523599;
			pose.pose_covariance[px4_msgs::msg::VehicleVisualOdometry::COVARIANCE_MATRIX_YAW_VARIANCE] = 0.0523599;

			pose.velocity_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
			pose.vx = NAN;
			pose.vy = NAN;
			pose.vz = NAN;
			pose.rollspeed = NAN;
			pose.pitchspeed = NAN;
			pose.yawspeed = NAN;
			pose.velocity_covariance[0] = NAN;
			pose.velocity_covariance[15] = NAN;
			pose.reset_counter = 0;
		}
		
};

int main(int argc, char * argv[])
{
	std::cout << "Starting Vicon PX4 Bridge node ... " << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<vicon_px4_bridge>());
	rclcpp::shutdown();
	return 0;
}
