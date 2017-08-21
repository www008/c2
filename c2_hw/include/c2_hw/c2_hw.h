#ifndef C2_HW_H
#define C2_HW_H

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <transmission_interface/transmission_parser.h>
#include <angles/angles.h>

// ros_control package
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <control_toolbox/pid.h>

#include <sensor_msgs/Imu.h>

#include <dynamic_reconfigure/server.h>
#include "c2_hw/DynamicParamConfig.h"

namespace c2_hw {

class WheelDriver;
	
class C2HW : public hardware_interface::RobotHW {
	
public:
	C2HW(ros::NodeHandle& nh);
	~C2HW();
	
	//配置管理
	virtual bool init();
	virtual void cleanup();
	//硬件管理
	virtual bool start();
	virtual void stop();
	//操作管理
	virtual bool read(const ros::Time time, const ros::Duration period);
	virtual void write(const ros::Time time, const ros::Duration period);

	//e_stop
	bool e_stop_active_, last_e_stop_active_;
	virtual void eStopActive(const bool active);

	friend WheelDriver;
	
protected:
	// Methods used to control a joint.
	enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

	void registerJointLimits(const std::string& joint_name,
	                         const hardware_interface::JointHandle& joint_handle,
	                         const ControlMethod ctrl_method,
	                         const ros::NodeHandle& joint_limit_nh,
	                         const urdf::Model *const urdf_model, int *const joint_type,
	                         double *const lower_limit, double *const upper_limit,
	                         double *const velocity_limit, double *const effort_limit);

	std::string getURDF(ros::NodeHandle &nh, std::string param_name) const;

	std::string printCommandHelper();
	std::string printStateHelper();

	// State
	ros::NodeHandle nh_;
	ros::NodeHandle mobile_controller_nh_;
	
	bool calibrated_;

	//------- ros-controls joint interface -------------
	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::PositionJointInterface position_joint_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;
	hardware_interface::EffortJointInterface effort_joint_interface_;
	// Joint limits interfaces - Saturation
	joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
	joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
	joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;
	// Joint limits interfaces - Soft limits
	joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
	joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
	joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

	//------ joint value ---------------
	std::size_t num_joints_;
	std::vector<std::string> joint_names_;
	std::vector<int> joint_types_;
	//states
	std::vector<double> joint_position_;
	std::vector<double> joint_velocity_;
	std::vector<double> joint_effort_;
	//limits
	std::vector<double> joint_position_lower_limits_;
	std::vector<double> joint_position_upper_limits_;
	std::vector<double> joint_velocity_limits_;
	std::vector<double> joint_effort_limits_;
	//commands
	std::vector<double> joint_position_command_;
	std::vector<double> joint_velocity_command_;
	std::vector<double> joint_effort_command_;

	std::vector<ControlMethod> joint_control_methods_;
	std::vector<control_toolbox::Pid> pid_controllers_;

	// Modes
	bool use_rosparam_joint_limits_;
	bool use_soft_limits_if_available_;

	//e_stop
	ros::Subscriber e_stop_sub_;
	std::vector<double> last_joint_position_command_;
	void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);
	
	
	//------------- drives -----------------
	WheelDriver* wheels;
	
	//---------- dynamic_reconfigure ------- 
	dynamic_reconfigure::Server<c2_hw::DynamicParamConfig> conf_server_;
	
	void dynamic_conf_cb(c2_hw::DynamicParamConfig &config, uint32_t level);
};

}
#endif