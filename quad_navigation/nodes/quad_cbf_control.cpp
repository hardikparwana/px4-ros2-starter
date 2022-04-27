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
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/external_actuator_controls.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class QuadController: public rclcpp::Node
{

	// Controller Variables
	Eigen::VectorXf state_pos = Eigen::VectorXf(9);
	Eigen::Quaternionf state_quat;
	Eigen::Vector3f state_omega;
	Eigen::Matrix3f state_R;
	Eigen::VectorXf setpoint_pos = Eigen::VectorXf(9);
	Eigen::Quaternionf setpoint_quat;
	float yaw_des = 0.0;
	Eigen::Vector3f setpoint_omega;
	Eigen::Matrix3f setpoint_R_prev;

	// Control Parameters
	float kx = 3.0;
	float kv = 0.0;//2.0;
	float kR = 0.1;//2.0;
	float kOmega = 0.0;//1.0;//0.5;
	float mass = 0.817;//1.0;
	float g = 9.81;
	float _hover_throttle = 0.55;//0.7;
	Eigen::Matrix3f J;
	float Ixx, Iyy, Izz;
	float torque_constant = 1000;

	// Time Variables
	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;
	std::atomic<uint64_t> timestamp_;
	float publish_freq;

	std::chrono::system_clock::time_point tp1 = std::chrono::system_clock::now(); 
	std::chrono::system_clock::time_point tp2 = std::chrono::system_clock::now(); 

	// Publishers
    rclcpp::Publisher<px4_msgs::msg::ExternalActuatorControls>::SharedPtr pub_actuator;

    // Subscribers
	rclcpp::TimerBase::SharedPtr sub_parameter;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr sub_angular_velocity;    
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr sub_pose_setpoint;

	public:
		QuadController(): Node("quad_controller"), count_(0)
		{
			// Parameters
			this->declare_parameter<float>("publish_freq", 200);
			this->declare_parameter<float>("kx", kx);
			this->declare_parameter<float>("kv", kv);
			this->declare_parameter<float>("kR", kR);
			this->declare_parameter<float>("kOmega", kOmega);
			this->declare_parameter<float>("mass", mass);
			this->declare_parameter<float>("Ixx", 0.03);
			this->declare_parameter<float>("Iyy", 0.03);
			this->declare_parameter<float>("Izz", 0.06);
			this->declare_parameter<float>("Hover Thrust", _hover_throttle);
			this->declare_parameter<float>("torque_constant",torque_constant);

			this->get_parameter("publish_freq",publish_freq);			
			this->get_parameter("kx", kx);
			this->get_parameter("kv", kv);
			this->get_parameter("kR", kR);
			this->get_parameter("kOmega", kOmega);
			this->get_parameter("mass", mass);
			this->get_parameter("Ixx", Ixx);
			this->get_parameter("Iyy", Iyy);
			this->get_parameter("Izz", Izz);
			this->get_parameter("Hover Thrust", _hover_throttle);
			this->get_parameter("torque_constant", torque_constant);
			J << Ixx, 0, 0,
			      0, Iyy, 0,
			      0, 0, Izz;
			setpoint_pos << 0,0,-0.6,0,0,0,0,0,0;
			setpoint_quat = Eigen::Quaternionf::Identity();
			setpoint_R_prev = Eigen::Matrix3f::Identity();
			setpoint_omega << 0,0,0;

			// Subscribers
			sub_position = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/vehicle_local_position/out",10, std::bind(&QuadController::position_callback, this, _1));
			sub_attitude = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/vehicle_attitude/out",10, std::bind(&QuadController::attitude_callback, this, _1));
			sub_angular_velocity = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/vehicle_angular_velocity/out",10, std::bind(&QuadController::angular_velocity_callback, this, _1));
			sub_pose_setpoint = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in",10, std::bind(&QuadController::setpoint_callback, this, _1));
			
			// Publishers
			pub_actuator = this->create_publisher<px4_msgs::msg::ExternalActuatorControls>("/fmu/external_actuator_controls/in",10);			
			timer_ = this->create_wall_timer(2ms, std::bind(&QuadController::controller_callback, this));

			sub_parameter = this->create_wall_timer(
      					1000ms, std::bind(&QuadController::update_parameters, this));

			timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
								[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
									timestamp_.store(msg->timestamp);
								});
		}

	private:

		void update_parameters()
		{
			this->get_parameter("publish_freq",publish_freq);			
			this->get_parameter("kx", kx);
			this->get_parameter("kv", kv);
			this->get_parameter("kR", kR);
			this->get_parameter("kOmega", kOmega);
			this->get_parameter("mass", mass);
			this->get_parameter("Ixx", Ixx);
			this->get_parameter("Iyy", Iyy);
			this->get_parameter("Izz", Izz);
			this->get_parameter("Hover Thrust", _hover_throttle);
			this->get_parameter("torque_constant",torque_constant);
		}

		void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) 
		{
			state_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->ax, msg->ay, msg->az;
		}

		void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) 
		{
			state_quat.w() = msg->q[0];
			state_quat.x() = msg->q[1];
			state_quat.y() = msg->q[2];
			state_quat.z() = msg->q[3];
		}

		void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) 
		{
			state_omega << msg->xyz[0], msg->xyz[1], msg->xyz[2];
		//	std::cout << state_omega << std::endl;
		}

		void setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) 
		{
			setpoint_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->acceleration[0], msg->acceleration[1], msg->acceleration[2];
			setpoint_quat.w() = 1.0; setpoint_quat.x() = 0.0; setpoint_quat.y() = 0.0; setpoint_quat.z() = 0.0;
			setpoint_omega << 0.0, 0.0, 0.0;
		}

		// Geometric Controller
		void controller_callback()
		{
			// NED Frame calculations

			// Translational Errors
			Eigen::Vector3f ex = state_pos.segment(0,3) - setpoint_pos.segment(0,3);
			Eigen::Vector3f ev = state_pos.segment(3,3) - setpoint_pos.segment(3,3);			
			
			// Rotational Errors
			Eigen::Matrix3f state_R = state_quat.normalized().toRotationMatrix();
			Eigen::Matrix3f state_Omega = vector_to_skew_symmetric(state_omega);

			Eigen::Matrix3f setpoint_R; 
			Eigen::Vector3f b3d = -( -kx * ex - kv * ev - mass * g * Eigen::Vector3f(0,0,1) + mass * setpoint_pos.segment(6,3) );
			if (b3d.norm()>0.01) {
				b3d = b3d.normalized();
			}
			else{
				b3d << 0.0 , 0.0 , 1.0;
				RCLCPP_INFO(this->get_logger(),"Division by zero. No thrust required");
			}
			Eigen::Vector3f b1d(cos(yaw_des), sin(yaw_des), 0);
			Eigen::Vector3f b2d = (b3d.cross(b1d)).normalized();
			b1d = (b2d.cross(b3d)).normalized();  //x1
			setpoint_R << b1d, b2d, b3d; 	

			tp2 = std::chrono::system_clock::now();
//			std::cout << "tp2 " << tp2 << "tp1: " << tp1 << std::endl;
			std::chrono::nanoseconds dt = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2-tp1);
			Eigen::Matrix3f setpoint_Rdot = (setpoint_R - setpoint_R_prev)/(dt.count()/1000000000.0);
			//std::cout << "Rd: " << setpoint_R << "\n Rd_prev: " << setpoint_R_prev << "\n Rdot: " << setpoint_Rdot << std::endl;
			setpoint_omega = skew_symmetric_to_vector( setpoint_R.transpose() * setpoint_Rdot );
			//std::cout << "dt: " << dt.count()/1000000000.0 << "setpoint_omega: " << setpoint_omega.transpose() << std::endl;
			tp1 = std::chrono::system_clock::now();
			setpoint_R_prev = setpoint_R;

			setpoint_omega << 0.0,0.0,0.0;

			setpoint_R << 1.0, 0.0, 0.0,
				      0.0, 0.0, 0.0,
				      0.0, 0.0, 1.0;

			Eigen::Matrix3f setpoint_Omega = vector_to_skew_symmetric(setpoint_omega);
			Eigen::Vector3f eR = 1.0/2 * skew_symmetric_to_vector( setpoint_R.transpose() * state_R - state_R.transpose() * setpoint_R  );
			Eigen::Vector3f e_omega = state_omega - state_R.transpose() * setpoint_R * setpoint_omega;

			//std::cout <<  "Yaw Roll Pitch " << state_R.eulerAngles(2,1,0).transpose() << " Angle Desired: " << setpoint_R.eulerAngles(2,1,0).transpose() << std::endl;
			//std::cout << "eR: " << eR.transpose() << " eOmega: " << e_omega.transpose() << std::endl;
			std::cout << "ex: " << ex.transpose() << " ev: " << ev.transpose() << "xd: " << setpoint_pos << std::endl;


			// Control input calculation
			float f =  -( -kx * ex - kv * ev - mass * g * Eigen::Vector3f(0,0,1) + mass * setpoint_pos.segment(6,3) ).transpose() * state_R * Eigen::Vector3f(0,0,1);
			Eigen::Vector3f M = - kR * eR - kOmega * e_omega + state_Omega * J * state_omega;// - J * ( state_Omega * state_R.transpose() * setpoint_R * setpoint_omega );// - state_R.transpose() * setpoint_R *  );
			Eigen::Vector3f angular_acceleration = J.inverse() * ( M - state_Omega * J * state_omega );

			// Normalize inputs
			f = f / mass / g * _hover_throttle;
			if (f<0){
				f = 0.01f;
			}

			//std::cout << "thrust: " << f << " M: " << angular_acceleration.transpose()/1000 << std::endl;

			px4_msgs::msg::ExternalActuatorControls external_actuator_controls;
			external_actuator_controls.roll = angular_acceleration(0)/torque_constant;
			external_actuator_controls.pitch = angular_acceleration(1)/torque_constant;
			external_actuator_controls.yaw = angular_acceleration(2)/torque_constant;
			external_actuator_controls.thrust = f;
			external_actuator_controls.timestamp = timestamp_.load();
			pub_actuator->publish(external_actuator_controls);
		}

		Eigen::Matrix3f vector_to_skew_symmetric(Eigen::Vector3f vec)
		{
			Eigen::Matrix3f mat;
			mat <<  0, - vec(2), vec(1),
					vec(2), 0, -vec(0),
					-vec(1), vec(0), 0;
			return mat;
		}

		Eigen::Vector3f skew_symmetric_to_vector(Eigen::MatrixXf mat)
		{
			Eigen::Vector3f vec;
			vec << ( mat(2,1)-mat(1,2) )/2, ( mat(0,2)-mat(2,0) )/2, ( mat(1,0)-mat(0,1) )/2;
			return vec;
		}

	
};

int main(int argc, char * argv[])
{
	std::cout << "Starting Quadrotor Controller node ..." << std::endl;
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<QuadController>());
	return 0;
}
