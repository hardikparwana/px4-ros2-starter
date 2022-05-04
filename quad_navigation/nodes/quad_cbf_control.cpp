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

	Eigen::VectorXf state_pos_prev = Eigen::VectorXf(9);
	Eigen::Quaternionf state_quat_prev;
	Eigen::Vector3f state_omega_prev;

	// Control Parameters
	float mass = 0.817;//1.5;//0.817;//1.0;
	float kx = 11.0*mass;//3.0;
	float kv = 6.0*mass;//2.0;//0.0;//2.0;
	float kR = 5.0;//10.0;//6.0;//0.1;//2.0;
	float kOmega = 2.0;//1.5;//0.0;//0.0;//1.0;//0.5;	
	float g = 9.81;
	float _hover_throttle = 0.7;//0.55;//0.7;
	Eigen::Matrix3f J;
	float Ixx, Iyy, Izz;
	float torque_constant = 1000.0;
	float tradeoff_thrust = 1.0;
	float tradeoff_roll = 1.0;
	float tradeoff_pitch = 1.0;
	float tradeoff_yaw = 1.0;
	float torque_max = 0.5;
	float thrust_max = 0.99;

	double state_pos_t;
	double state_pos_t_prev;
	double state_quat_t;
	double state_quat_t_prev;
	double state_omega_t;
	double state_omega_t_prev;

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
			this->declare_parameter<float>("tradeoff_thrust",tradeoff_thrust);
			this->declare_parameter<float>("tradeoff_roll",tradeoff_roll);
			this->declare_parameter<float>("tradeoff_pitch",tradeoff_pitch);
			this->declare_parameter<float>("tradeoff_yaw",tradeoff_yaw);
			this->declare_parameter<float>("torque_max",torque_max);
			this->declare_parameter<float>("thrust_max",thrust_max);

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
			this->get_parameter<float>("tradeoff_thrust",tradeoff_thrust);
			this->get_parameter<float>("tradeoff_roll",tradeoff_roll);
			this->get_parameter<float>("tradeoff_pitch",tradeoff_pitch);
			this->get_parameter<float>("tradeoff_yaw",tradeoff_yaw);
			this->get_parameter<float>("torque_max",torque_max);
			this->get_parameter<float>("thrust_max",thrust_max);
			J << Ixx, 0, 0,
			      0, Iyy, 0,
			      0, 0, Izz;
			setpoint_pos << 0,0,-0.6,0,0,0,0,0,0;
			setpoint_quat = Eigen::Quaternionf::Identity();
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
      					500ms, std::bind(&QuadController::update_parameters, this));

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
			this->get_parameter<float>("tradeoff_thrust",tradeoff_thrust);
			this->get_parameter<float>("tradeoff_roll",tradeoff_roll);
			this->get_parameter<float>("tradeoff_pitch",tradeoff_pitch);
			this->get_parameter<float>("tradeoff_yaw",tradeoff_yaw);
			this->get_parameter<float>("torque_max",torque_max);
			this->get_parameter<float>("thrust_max",thrust_max);
		}

		void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) 
		{
			state_pos_t_prev = state_pos_t;
			state_pos_prev = state_pos;
			state_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->ax, msg->ay, msg->az;
			state_pos_t = msg->timestamp;
			// std::cout << "pos: " << state_pos.transpose() << " prev: " << state_pos_prev.transpose() << std::endl;
			// std::cout << "pos time new: " << state_pos_t << "time old: " << state_pos_t_prev << "diff: " << (state_omega_t-state_pos_t_prev) << std::endl;
			
		}

		void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) 
		{
			state_quat_prev = state_quat;
			state_quat_t_prev = state_quat_t;

			state_quat.w() = msg->q[0];
			state_quat.x() = msg->q[1];
			state_quat.y() = msg->q[2];
			state_quat.z() = msg->q[3];
			state_quat_t = msg->timestamp;
		}

		void angular_velocity_callback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) 
		{
			// std::cout << "time old: " << state_omega_t << std::endl;
			state_omega_t_prev = state_omega_t;
			state_omega_prev = state_omega;

			state_omega << msg->xyz[0], msg->xyz[1], msg->xyz[2];
			state_omega_t = msg->timestamp;

			// std::cout << "time: " << msg->timestamp << "angular velocity: " << state_omega.transpose() << "omega prev: " << state_omega_prev << std::endl;
			// std::cout << "time new: " << state_omega_t << "time old: " << state_omega_t_prev << "diff: " << (state_omega_t-state_omega_t_prev) << std::endl;
		}

		void setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) 
		{
			setpoint_pos << msg->x, msg->y, msg->z, msg->vx, msg->vy, msg->vz, msg->acceleration[0], msg->acceleration[1], msg->acceleration[2];
			// setpoint_quat.w() = cos(msg->yaw/2.0); setpoint_quat.x() = 0.0; setpoint_quat.y() = 0.0; setpoint_quat.z() = sin(msg->yaw/2.0);
			// setpoint_omega << 0.0, 0.0, 0.0;
			yaw_des = msg->yaw;
		}

		// Geometric Controller
		void controller_callback()
		{
			// NED Frame calculations

			////////// Translational Errors 
			Eigen::Vector3f ex = state_pos.segment(0,3) - setpoint_pos.segment(0,3);
			Eigen::Vector3f ev = state_pos.segment(3,3) - setpoint_pos.segment(3,3);	

			Eigen::Vector3f ex_prev = state_pos_prev.segment(0,3) - setpoint_pos.segment(0,3);
			Eigen::Vector3f ev_prev = state_pos_prev.segment(3,3) - setpoint_pos.segment(3,3);		
			///////////////////////////////////////////////	
			
			////////// Rotational Errors
			Eigen::Matrix3f state_R = state_quat.normalized().toRotationMatrix();
			Eigen::Matrix3f state_Omega = vector_to_skew_symmetric(state_omega);
			//////////////////////////////////////

			// Desired Rotation Matrix
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
			//////////////////////////////////////

			// Desired Previous Rotation Matrix based on current setpoint
			Eigen::Matrix3f setpoint_R_prev; 
			Eigen::Vector3f b3d_prev = -( -kx * ex_prev - kv * ev_prev - mass * g * Eigen::Vector3f(0,0,1) + mass * setpoint_pos.segment(6,3) );
			if (b3d_prev.norm()>0.01) {
				b3d_prev = b3d_prev.normalized();
			}
			else{
				b3d_prev << 0.0 , 0.0 , 1.0;
				RCLCPP_INFO(this->get_logger(),"Division by zero. No thrust required");
			}
			Eigen::Vector3f b1d_prev(cos(yaw_des), sin(yaw_des), 0);
			Eigen::Vector3f b2d_prev = (b3d_prev.cross(b1d_prev)).normalized();
			b1d_prev = (b2d_prev.cross(b3d_prev)).normalized();  //x1
			setpoint_R_prev << b1d_prev, b2d_prev, b3d_prev; 
			/////////////////////////////////////


			// tp2 = std::chrono::system_clock::now();
			// std::cout << "tp2 " << tp2 << "tp1: " << tp1 << std::endl;
			// std::chrono::nanoseconds dt = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2-tp1);
			// Eigen::Matrix3f setpoint_Rdot = (setpoint_R - setpoint_R_prev)/(dt.count()/1000000000.0);
			float dt = (state_pos_t - state_pos_t_prev)/1000000.0;
			Eigen::Matrix3f setpoint_Rdot = (setpoint_R - setpoint_R_prev)/dt;
			setpoint_omega = skew_symmetric_to_vector( setpoint_R.transpose() * setpoint_Rdot );
			// std::cout << "dt: " << dt << "setpoint_omega: " << setpoint_omega.transpose() << std::endl;

			//std::cout << "Rd: " << setpoint_R << "\n Rd_prev: " << setpoint_R_prev << "\n Rdot: " << setpoint_Rdot << std::endl;
			//std::cout << "dt: " << dt.count()/1000000000.0 << "setpoint_omega: " << setpoint_omega.transpose() << std::endl;
			// tp1 = std::chrono::system_clock::now();

			// Test Altitude
			// setpoint_omega << 0.0,0.0,0.0;
			// setpoint_R << 1.0, 0.0, 0.0,
			// 	      0.0, 0.0, 0.0,
			// 	      0.0, 0.0, 1.0;

			Eigen::Matrix3f setpoint_Omega = vector_to_skew_symmetric(setpoint_omega);
			Eigen::Vector3f eR = 1.0/2 * skew_symmetric_to_vector( setpoint_R.transpose() * state_R - state_R.transpose() * setpoint_R  );
			Eigen::Vector3f e_omega = state_omega - state_R.transpose() * setpoint_R * setpoint_omega;

			//std::cout <<  "Yaw Roll Pitch " << state_R.eulerAngles(2,1,0).transpose() << " Angle Desired: " << setpoint_R.eulerAngles(2,1,0).transpose() << std::endl;
			//std::cout << "eR: " << eR.transpose() << " eOmega: " << e_omega.transpose() << std::endl;
			// std::cout << "ex: " << ex.transpose() << " ev: " << ev.transpose() << "xd: " << setpoint_pos << std::endl;

			// Control input calculation
			Eigen::Vector3f setpoint_omega_dot;
			setpoint_omega_dot << 0, 0, 0;
			float f =  -( -kx * ex - kv * ev - mass * g * Eigen::Vector3f(0,0,1) + mass * setpoint_pos.segment(6,3) ).transpose() * state_R * Eigen::Vector3f(0,0,1);
			Eigen::Vector3f M = - kR * eR - kOmega * e_omega + state_Omega * J * state_omega - J * ( state_Omega * state_R.transpose() * setpoint_R * setpoint_omega - state_R.transpose() * setpoint_R * setpoint_omega_dot );
			Eigen::Vector3f angular_acceleration = J.inverse() * ( M - state_Omega * J * state_omega ) / torque_constant;

			// Normalize inputs
			f = f / mass / g * _hover_throttle;
			if (f<0){
				f = 0.01f;
			}

			if (f>thrust_max){
				f = thrust_max;
				std::cout << "Passed Thrust Limit. Constraining.................." << std::endl;
			}	

			//std::cout << "thrust: " << f << " M: " << angular_acceleration.transpose()/1000 << std::endl;
			if (angular_acceleration.norm() > torque_max){
				std::cout << "Passed Torque Limit. Constraining.................." << std::endl;
				angular_acceleration = torque_max * angular_acceleration.normalized();
			}				

			px4_msgs::msg::ExternalActuatorControls external_actuator_controls;
			external_actuator_controls.roll = angular_acceleration(0);
			external_actuator_controls.pitch = angular_acceleration(1);
			external_actuator_controls.yaw = angular_acceleration(2);
			external_actuator_controls.thrust = f;
			external_actuator_controls.tradeoff_thrust = tradeoff_thrust;
			external_actuator_controls.tradeoff_roll = tradeoff_roll;
			external_actuator_controls.tradeoff_pitch = tradeoff_pitch;
			external_actuator_controls.tradeoff_yaw = tradeoff_yaw;
			external_actuator_controls.timestamp = state_omega_t;//  timestamp_.load();
			// std::cout << "previously published time: " << timestamp_.load()/1000.0 << std::endl;
			pub_actuator->publish(external_actuator_controls);

			// std::cout << "tradeoff: " << external_actuator_controls.tradeoff << std::endl;
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
