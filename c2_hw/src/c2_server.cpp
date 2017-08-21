#include <stdexcept>
#include <cmath>
#include <time.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Bool.h>

#include "c2_hw/wheel_driver.h"
#include "c2_hw/c2_hw.h"

namespace c2_hw {

double clamp(const double val, const double min_val, const double max_val) {
	return std::min(std::max(val, min_val), max_val);
}

C2HW::C2HW(ros::NodeHandle& nh): nh_(nh), mobile_controller_nh_(nh.getNamespace() + "/mobile_controller"), calibrated_(false) {
	wheels = new WheelDriver(mobile_controller_nh_);
}

C2HW::~C2HW() {
	delete wheels;
}

std::string C2HW::getURDF(ros::NodeHandle &nh, std::string param_name) const {
	std::string urdf_string;

	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok()) {
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name)) {
			ROS_INFO_STREAM("Waiting for model URDF on the ROS param server at location1: " <<
			                      nh.getNamespace() << "--" << search_param_name);
			nh.getParam(search_param_name, urdf_string);
		} else {
			ROS_INFO_STREAM("Waiting for model URDF on the ROS param server at location2: " <<
			                      nh.getNamespace() << "--" << param_name);
			nh.getParam(param_name, urdf_string);
		}

		usleep(100000);
	}

	return urdf_string;
}

//初始化配置
bool C2HW::init() {
	urdf::Model urdf_model;
	std::vector<transmission_interface::TransmissionInfo> transmissions;

	//---- load URDF
	const std::string urdf_string = getURDF(nh_,"robot_description");
	if (!urdf_model.initString(urdf_string)) {
		ROS_ERROR( "Unable to load URDF model");
		return false;
	}
	ROS_DEBUG("c2_server load urdf.");
	if (!transmission_interface::TransmissionParser::parse(urdf_string, transmissions)) {
		ROS_ERROR("Error parsing URDF.\n");
		return false;
	}
	ROS_DEBUG("c2_server load transmissions.");
	
	//----- init joint --------------
	const ros::NodeHandle joint_limit_nh(nh_);

	num_joints_ = transmissions.size();
	joint_names_.resize(num_joints_);
	joint_types_.resize(num_joints_);

	joint_position_.resize(num_joints_);
	joint_velocity_.resize(num_joints_);
	joint_effort_.resize(num_joints_);

	joint_position_lower_limits_.resize(num_joints_);
	joint_position_upper_limits_.resize(num_joints_);
	joint_velocity_limits_.resize(num_joints_);
	joint_effort_limits_.resize(num_joints_);

	joint_position_command_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);
	joint_effort_command_.resize(num_joints_);

	joint_control_methods_.resize(num_joints_);
	pid_controllers_.resize(num_joints_);

	ROS_DEBUG("c2_server start init joints(%lu)....", num_joints_);
	// Initialize interfaces for each joint
	for(unsigned int j=0; j < num_joints_; j++) {
		ROS_DEBUG("c2_server init joints(%u)....", j);
		// Check that this transmission has one joint
		if(transmissions[j].joints_.size() == 0) {
			ROS_WARN_STREAM("Transmission " << transmissions[j].name_
			                      << " has no associated joints.");
			continue;
		} else if(transmissions[j].joints_.size() > 1) {
			ROS_WARN_STREAM("Transmission " << transmissions[j].name_
			                      << " has more than one joint. Currently the default robot hardware simulation "
			                      << " interface only supports one.");
			continue;
		}

		std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
		if (joint_interfaces.empty() &&
		        !(transmissions[j].actuators_.empty()) &&
		        !(transmissions[j].actuators_[0].hardware_interfaces_.empty())) {
			// TODO: Deprecate HW interface specification in actuators in ROS J
			joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
			ROS_WARN_STREAM( "The <hardware_interface> element of tranmission " <<
			                      transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
			                      "The transmission will be properly loaded, but please update " <<
			                      "your robot model to remain compatible with future versions of the plugin.");
		}
		if (joint_interfaces.empty()) {
			ROS_WARN_STREAM( "Joint " << transmissions[j].joints_[0].name_ <<
			                      " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
			                      "Not adding it to the robot hardware simulation.");
			continue;
		} else if (joint_interfaces.size() > 1) {
			ROS_WARN_STREAM( "Joint " << transmissions[j].joints_[0].name_ <<
			                      " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
			                      "Currently the default robot hardware simulation interface only supports one. Using the first entry");
			//continue;
		}
	
		// Add data from transmission
		joint_names_[j] = transmissions[j].joints_[0].name_;
		joint_position_[j] = 1.0;
		joint_velocity_[j] = 0.0;
		joint_effort_[j] = 1.0;  // N/m for continuous joints
		joint_effort_command_[j] = 0.0;
		joint_position_command_[j] = 0.0;
		joint_velocity_command_[j] = 0.0;

		const std::string& hardware_interface = joint_interfaces.front();

		// Debug
		ROS_DEBUG_STREAM("Loading joint '" << joint_names_[j] << "' of type '" << hardware_interface << "'");

		// Create joint state interface for all joints
		joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
		        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

		// Decide what kind of command interface this actuator/joint has
		hardware_interface::JointHandle joint_handle;
		if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface") {
			// Create effort joint interface
			joint_control_methods_[j] = EFFORT;
			joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[j]), &joint_effort_command_[j]);
			effort_joint_interface_.registerHandle(joint_handle);
		} else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface") {
			// Create position joint interface
			joint_control_methods_[j] = POSITION;
			joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[j]), &joint_position_command_[j]);
			position_joint_interface_.registerHandle(joint_handle);
		} else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface") {
			// Create velocity joint interface
			joint_control_methods_[j] = VELOCITY;
			joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[j]), &joint_velocity_command_[j]);
			velocity_joint_interface_.registerHandle(joint_handle);
		} else {
			ROS_FATAL_STREAM("No matching hardware interface found for '"
			                       << hardware_interface << "' while loading interfaces for " << joint_names_[j] );
			return false;
		}

		if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
			ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_names_[j] << "'.");
		}

		registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
		                    joint_limit_nh, &urdf_model,
		                    &joint_types_[j], &joint_position_lower_limits_[j], &joint_position_upper_limits_[j],
		                    &joint_velocity_limits_[j], &joint_effort_limits_[j]);

		// Initialize the PID controller
		if (joint_control_methods_[j] != EFFORT) {
			//If no PID gain values are found, use joint->SetAngle() or joint->SetParam("vel") to control the joint.
			const ros::NodeHandle nh(nh_, "/ros_control/pid_gains/" + joint_names_[j]);
			if (pid_controllers_[j].init(nh, true)) {
				switch (joint_control_methods_[j]) {
				case POSITION:
					joint_control_methods_[j] = POSITION_PID;
					break;
				case VELOCITY:
					joint_control_methods_[j] = VELOCITY_PID;
					break;
				}
			} else {
				// joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
				// going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
				// going to be called.
				//joint->SetParam("fmax", 0, joint_effort_limits_[j]);
			}
		}
	}
	
	// TODO: Set up gravity compensator
	// Write kdl_ros_integration package which handles kdl/urdf/ros_control? interfaces

	// Register ros-controls interfaces
	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
	registerInterface(&velocity_joint_interface_);
	registerInterface(&effort_joint_interface_);
	ROS_DEBUG("c2_server register joints over.");
	
	if(!wheels->configure(*this)) {
		ROS_ERROR("c2_server wheels configure failed!");
		return false;
	}
	
	ROS_DEBUG("c2_server wheels configure over.");
	return true;
}

// Emergency stop callback
void C2HW::eStopCB(const std_msgs::BoolConstPtr& e_stop_active) {
	eStopActive(e_stop_active->data);
}

//清理配置
void C2HW::cleanup() {
	wheels->cleanup();
	e_stop_sub_.shutdown();
}

bool C2HW::start() {
	
	//---------- publish topic ---------------
	// wheels
	wheels->init();
	wheels->setZero();
	

	//---------- subscribe topic ---------------

	// Initialize the emergency stop.
	e_stop_active_ = false;
	last_e_stop_active_ = false;
	std::string e_stop_topic;
	if (nh_.getParam("eStopTopic",e_stop_topic)) {
		e_stop_sub_ = nh_.subscribe(e_stop_topic, 1, &C2HW::eStopCB, this);
	} else {
		ROS_FATAL("No rosparam: eStopTopic" );
		return false;
	}

	// Wait for the system to become active
	//this->wait_for_mode(barrett::SafetyModule::ACTIVE);

	dynamic_reconfigure::Server<c2_hw::DynamicParamConfig>::CallbackType f;
	f = boost::bind(&C2HW::dynamic_conf_cb, this, _1, _2);
	conf_server_.setCallback(f);
	return true;
}

void C2HW::stop() {
	wheels->setZero();

	// Wait for the system to become active
	//this->wait_for_mode(barrett::SafetyModule::IDLE);
}

//读robot状态
bool C2HW::read(const ros::Time time, const ros::Duration period) {
	for(unsigned int j=0; j < num_joints_; j++) {
		if (joint_types_[j] == urdf::Joint::PRISMATIC) {
			//joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
		} else {
			//joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], sim_joints_[j]->GetAngle(0).Radian());
		}
		//joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
		//joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
	}
	
		
	wheels->readState();
	
	return true;
}

void C2HW::write(const ros::Time time, const ros::Duration period) {
	// If the eStop is active, joints controlled by position commands will maintain their positions.
	if (e_stop_active_) {
		if (!last_e_stop_active_) {
			last_joint_position_command_ = joint_position_;
			last_e_stop_active_ = true;
		}
		joint_position_command_ = last_joint_position_command_;
	} else {
		last_e_stop_active_ = false;
	}

	pos_jnt_sat_interface_.enforceLimits(period);
	vel_jnt_sat_interface_.enforceLimits(period);
	eff_jnt_sat_interface_.enforceLimits(period);

	pos_jnt_soft_limits_.enforceLimits(period);
	vel_jnt_soft_limits_.enforceLimits(period);
	eff_jnt_soft_limits_.enforceLimits(period);


	for(unsigned int j=0; j < num_joints_; j++) {
		switch (joint_control_methods_[j]) {
		case EFFORT: {
			const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
			//sim_joints_[j]->SetForce(0, effort);
		}
		break;

		case POSITION:
			//sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
			break;

		case POSITION_PID: {
			double error;
			switch (joint_types_[j]) {
			case urdf::Joint::REVOLUTE:
				angles::shortest_angular_distance_with_limits(joint_position_[j], joint_position_command_[j],
				        joint_position_lower_limits_[j], joint_position_upper_limits_[j], error);
				break;
			case urdf::Joint::CONTINUOUS:
				error = angles::shortest_angular_distance(joint_position_[j], joint_position_command_[j]);
				break;
			default:
				error = joint_position_command_[j] - joint_position_[j];
			}

			const double effort_limit = joint_effort_limits_[j];
			const double effort = clamp(pid_controllers_[j].computeCommand(error, period), -effort_limit, effort_limit);
			//sim_joints_[j]->SetForce(0, effort);
		}
		break;

		case VELOCITY:
			//sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
			break;

		case VELOCITY_PID:
			double error;
			if (e_stop_active_)
				error = -joint_velocity_[j];
			else
				error = joint_velocity_command_[j] - joint_velocity_[j];
			const double effort_limit = joint_effort_limits_[j];
			const double effort = clamp(pid_controllers_[j].computeCommand(error, period), -effort_limit, effort_limit);
			//sim_joints_[j]->SetForce(0, effort);
			break;
		}
	}
	
	wheels->writeCmd();
}


void C2HW::eStopActive(const bool active) {
	e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void C2HW::registerJointLimits(const std::string& joint_name,
                               const hardware_interface::JointHandle& joint_handle,
                               const ControlMethod ctrl_method,
                               const ros::NodeHandle& joint_limit_nh,
                               const urdf::Model *const urdf_model, int *const joint_type,
                               double *const lower_limit, double *const upper_limit,
                               double *const velocity_limit, double *const effort_limit) {
	*joint_type = urdf::Joint::UNKNOWN;
	*lower_limit = -std::numeric_limits<double>::max();
	*upper_limit = std::numeric_limits<double>::max();
	*velocity_limit = std::numeric_limits<double>::max();
	*effort_limit = std::numeric_limits<double>::max();

	joint_limits_interface::JointLimits limits;
	bool has_limits = false;
	joint_limits_interface::SoftJointLimits soft_limits;
	bool has_soft_limits = false;

	if (urdf_model != NULL) {
		const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
		if (urdf_joint != NULL) {
			*joint_type = urdf_joint->type;
			// Get limits from the URDF file.
			if (joint_limits_interface::getJointLimits(urdf_joint, limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
				has_soft_limits = true;
		}
	}
	// Get limits from the parameter server.
	if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
		has_limits = true;

	if (!has_limits)
		return;

	if (*joint_type == urdf::Joint::UNKNOWN) {
		// Infer the joint type.

		if (limits.has_position_limits) {
			*joint_type = urdf::Joint::REVOLUTE;
		} else {
			if (limits.angle_wraparound)
				*joint_type = urdf::Joint::CONTINUOUS;
			else
				*joint_type = urdf::Joint::PRISMATIC;
		}
	}

	if (limits.has_position_limits) {
		*lower_limit = limits.min_position;
		*upper_limit = limits.max_position;
	}
	if (limits.has_velocity_limits)
		*velocity_limit = limits.max_velocity;
	if (limits.has_effort_limits)
		*effort_limit = limits.max_effort;

	if (has_soft_limits) {
		switch (ctrl_method) {
		case EFFORT: {
			const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
			eff_jnt_soft_limits_.registerHandle(limits_handle);
		}
		break;
		case POSITION: {
			const joint_limits_interface::PositionJointSoftLimitsHandle	limits_handle(joint_handle, limits, soft_limits);
			pos_jnt_soft_limits_.registerHandle(limits_handle);
		}
		break;
		case VELOCITY: {
			const joint_limits_interface::VelocityJointSoftLimitsHandle	limits_handle(joint_handle, limits, soft_limits);
			vel_jnt_soft_limits_.registerHandle(limits_handle);
		}
		break;
		}
	} else {
		switch (ctrl_method) {
		case EFFORT: {
			const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
			eff_jnt_sat_interface_.registerHandle(sat_handle);
		}
		break;
		case POSITION: {
			const joint_limits_interface::PositionJointSaturationHandle	sat_handle(joint_handle, limits);
			pos_jnt_sat_interface_.registerHandle(sat_handle);
		}
		break;
		case VELOCITY: {
			const joint_limits_interface::VelocityJointSaturationHandle	sat_handle(joint_handle, limits);
			vel_jnt_sat_interface_.registerHandle(sat_handle);
		}
		break;
		}
	}
}

void C2HW::dynamic_conf_cb(c2_hw::DynamicParamConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: 	%f", config.speed_scale );
}


std::string C2HW::printStateHelper() {
	std::stringstream ss;
	std::cout.precision(15);

	for (std::size_t i = 0; i < num_joints_; ++i) {
		ss << "j" << i << "(" << joint_names_[i] << "): " << std::fixed << joint_position_[i] << "\t ";
		ss << std::fixed << joint_velocity_[i] << "\t ";
		ss << std::fixed << joint_effort_[i] << std::endl;
	}
	return ss.str();
}

std::string C2HW::printCommandHelper() {
	std::stringstream ss;
	std::cout.precision(15);
	ss << "    position     velocity         effort  \n";
	for (std::size_t i = 0; i < num_joints_; ++i) {
		ss << "j" << i << "(" << joint_names_[i] << "): " << std::fixed << joint_position_command_[i] << "\t ";
		ss << std::fixed << joint_velocity_command_[i] << "\t ";
		ss << std::fixed << joint_effort_command_[i] << std::endl;
	}
	return ss.str();
}

}

//********** Main ************************
bool g_quit = false;

void quitRequested(int sig) {
	g_quit = true;
}

int main( int argc, char** argv ) {
	std::cout << "c2_server main ...";
	ROS_INFO_NAMED("c2_server","c2_server MAIN ...");
	// Initialize ROS
	ros::init(argc, argv, "c2_server", ros::init_options::NoSigintHandler);

	// Add custom signal handlers
	signal(SIGTERM, quitRequested);
	signal(SIGINT, quitRequested);
	signal(SIGHUP, quitRequested);

	ros::NodeHandle nh;
	c2_hw::C2HW c2_robot(nh);

	ROS_DEBUG("c2_server init ready...");
	if(!c2_robot.init()) {
		ROS_FATAL_NAMED("c2_server","Could not initialize robot hardware_interface");
		return -1;
	}

	// Timer variables
	struct timespec ts = {0,0};
	if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
		ROS_FATAL("Failed to poll realtime clock!");
	}

	ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
	ros::Duration period(1.0);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(nh, "loop_rate", 2);

	ROS_DEBUG("c2_server start ready...");
	bool c2_ok = false;
	while(!g_quit && !c2_ok) {
		if(!c2_robot.start()) {
			ROS_ERROR("Could not start Robot!");
		} else {
			ros::Duration(1.0).sleep();

			if(!c2_robot.read(now, period)) {
				ROS_ERROR("Could not read from Robot!");
			} else {
				c2_ok = true;
			}
		}

		ros::Duration(1.0).sleep();
	}


	// Construct the controller manager
	controller_manager::ControllerManager manager(&c2_robot, nh);

	ROS_DEBUG("c2_server runing start...");
	uint32_t count = 0;
	// Run as fast as possible
	while( !g_quit ) {
		// Get the time / period
		if (!clock_gettime(CLOCK_REALTIME, &ts)) {
			now.sec = ts.tv_sec;
			now.nsec = ts.tv_nsec;
			period = now - last;
			last = now;
		} else {
			ROS_FATAL("Failed to poll realtime clock!");
			break;
		}
		
		// Read the state from the Robot
		if(!c2_robot.read(now, period)) {
			ROS_ERROR("Could not read from Robot!");
			g_quit=true;
			break;
		}

		// Compute the controller commands
		bool reset_ctrlrs;
		if (c2_robot.e_stop_active_) {
			reset_ctrlrs = false;
		} else {
			if (c2_robot.last_e_stop_active_) {
				reset_ctrlrs = true;
			} else {
				reset_ctrlrs = false;
			}
		}
		// Update the controllers
		manager.update(now, period, reset_ctrlrs);

		// Write the command to the WAM
		c2_robot.write(now, period);

		if(count++ > 1000) {
			if(publisher.trylock()) {
				count = 0;
				publisher.msg_.data = period;
				publisher.unlockAndPublish();
			}
		}
	}

	publisher.stop();

	std::cerr<<"Stpping spinner..."<<std::endl;
	spinner.stop();

	std::cerr<<"Stopping C2..."<<std::endl;
	c2_robot.stop();

	std::cerr<<"Cleaning up C2..."<<std::endl;
	c2_robot.cleanup();

	std::cerr<<"Over!"<<std::endl;

	return 0;
}
