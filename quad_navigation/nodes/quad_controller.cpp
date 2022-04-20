#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/timesync.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/actuator_controls.hpp>

using namespace std::chrono_literals;
using namespace Eigen;

class QuadController: public rclcpp::Node
{

	// Controller Variables
	VectorXf state_pos(9);
	Quaternionf state_quat;
	Vector3f state_omega;
	Matrix3f state_R;
	VectorXf setpoint_pos(9);
	Quaternionf setpoint_quat;
	Vector3f setpoint_omega;

	// Control Parameters
	float kx = 1.0;
	float kv = 1.0;
	float kR = 1.0;
	float kOmega = 1.0;
	float mass = 1.0;
	float g = 9.81;
	Matrix3f J;
	float Ixx, Iyy, Izz;

	// Time Variables
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;
	float publish_freq;

	// Publishers
    rclcpp::Publisher<>::SharedPtr pub_inputs;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_angular_velocity;    
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr sub_pose_setpoint;

	public:
		QuadController(): Node("quad_controller"), count_(0)
		{
			// Parameters
			this->declare_parameter<std::string>("publish_freq", 200);
			this->declare_parameter<float>("kx", 1.0);
			this->declare_parameter<float>("kv", 1.0);
			this->declare_parameter<float>("kR", 1.0);
			this->declare_parameter<float>("kOmega", 1.0);
			this->declare_parameter<float>("mass", 1.0);
			this->declare_parameter<float>("Ixx", 0.05);
			this->declare_parameter<float>("Iyy", 0.05);
			this->declare_parameter<float>("Izz", 0.1);

			this->get_parameter("publish_freq",publish_freq);			
			this->get_parameter("kx", kx);
			this->get_parameter("kv", ky);
			this->get_parameter("kR", kR);
			this->get_parameter("kOmega", kOmega);
			this->get_parameter("mass", mass);
			this->get_parameter("Ixx", Ixx);
			this->get_parameter("Iyy", Iyy);
			this->get_parameter("Izz", Izz);
			J << Ixx, 0, 0,
			      0, Iyy, 0,
			      0, 0, Izz;

			// Subscribers
			sub_pose = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/vehicle_local_position/out",10, std::bind(&QuadController::position_callback, this, 1));
			sub_attitude = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/vehicle_attitude/out",10, std::bind(&QuadController::attitude_callback, this, 1));
			sub_angular_velocity = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/vehicle_angular_velocity/out",10, std::bind(&QuadController::angular_velocity_callback, this, 1));
			sub_pose_setpoint = this->create_subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>("/fmu/vehicle_local_position/out",10, std::bind(&QuadController::setpoint_callback, this, 1));
			
			// Publishers
			pub_actuator = this->create_publisher<px4_msgs::msg::ActuatorControls>("/fmu/vehicle_visual_odometry/in");			
			timer_ = this->create_wall_timer(publish_freq, std::bind(&QuadController::controller_callback, this));

			timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(timesync_sub_name, 10,
								[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
									timestamp_.store(msg->timestamp);
								});
		}

	private:
		void pose_callback(const px4_msgs::msgs::VehicleLocalPosition::SharedPtr msg) const
		{
			state_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->ax, msg->ay, msg->az;
		}

		void attitude_callback(const px4_msgs::msgs::VehicleAttitude::SharedPtr msg) const
		{
			state_quat.w() = msg->q[0];
			state_quat.x() = msg->q[1];
			state_quat.y() = msg->q[2];
			state_quat.z() = msg->q[3];
		}

		void angular_velocity_callback(const px4::msgs::VehicleAngularVelocity::SharedPtr msg) const
		{
			state_omega << msg->xyz[0], msg->xyz[1], msg->xyz[2];
		}

		void setpoint_callback(const px4_msgs::msgs::VehicleLocalPositionSetpoint::SharedPtr msg) const
		{
			setpoint_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, , msg->ax, msg->ay, msg->az;
			setpoint_quat.w() = msg->q[0]; setpoint_quat.x() = msg->q[1]; setpoint_quat.y() = msg->q[2]; setpoint_quat.z() = msg->q[3];
			setpoint_omega << msg->wx[0], msg->wy[1], msg->wz[2];
		}

		// Geometric Controller
		void controller_callback()
		{
			// NED Frame calculations

			Matrix3f state_R = state_quat.normalized().toRotationMatrix();
			Matrix3f setpoint_R = setpoint_quat.normalized().toRotationMatrix();
			Matrix3f state_Omega = vector_to_skew_symmetric(state_omega);
			Matrix3f setpoint_Omega = vector_to_skew_symmetric(setpoint_omega);

			// Translational Errors
			Vector3f ex = pos.segment(0,3) - setpoint_pos.segment(0,3);
			Vector3f ev = pos.segment(3,3) - setpoint_pose.segment(3,3);

			// Rotational Errors
			Vector3f eR = 1/2 * skew_symmetric_to_vector( setpoint_R.transpose() * state_R - state_R.transpose() * setpoint_R  );
			Matrix3f e_omega = state_omega - state_R.transpose() * setpoint_R.transpose() * setpoint_omega;

			// Controol input calculation
			float f = -( -kx * state - kv * ev - mass * g * Vector3f(0,0,1) + mass * setpoint_pos.segment(6,3) ) * state_R * Vector3f(0,0,1);
			Vector3f M = - kR * eR - kOmega * e_omega + state_Omega * J * state_omega - J * ( state_Omega * state_R.transpose() * setpoint_R * setpoint_omega );// - state_R.transpose() * setpoint_R *  );

			Vector3f angular_acceleration = J.inverse() * ( M - state_Omega * J * state_omega );

			// Normalize inputs
			f = f / mass / g;

			px4_msgs::msgs::ActuatorControls actuators_control;
			actuators_control[px4_msgs::msg::ActuatorControls::INDEX_ROLL] = angular_acceleration(1);
			actuators_control[px4_msgs::msg::ActuatorControls::INDEX_PITCH] = angular_acceleration(0);
			actuators_control[px4_msgs::msg::ActuatorControls::INDEX_YAW] = -angular_acceleration(3);
			actuators_control[px4_msgs::msg::ActuatorControls::INDEX_THROTTLE] = f;
			actuators_control.timestamp = timestamp_.load();
			actuators_control.timestamp_sample = timestamp_.load();
			pub_actuator->publish(actuators_control);
		}

		Matrix3f vector_to_skew_symmetric(Vector3f vec)
		{
			Matrix3f mat;
			mat <<  0, - vec(2), vec(1),
					vec(2), 0, -vec(0),
					-vec(1), vec(0), 0;
			return mat;
		}

		Vector3f skew_symmetric_to_vector(MatrixXf mat)
		{
			Vector3f vec;
			vec << ( mat(2,1)-mat(1,2) )/2, ( mat(0,2)-mat(2,0) )/2, ( mat(1,0)-mat(0,1) )/2;
			return vec;
		}

	
}

int main(int argv, char *argv[])
{
	RCLCPP_INFO(this->get_logger(),"Starting Quadrotor Controller node ...");
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<>());
	return 0;
}