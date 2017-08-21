#include <stdexcept>
#include <cmath>
#include <time.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
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

namespace hw_control_template {

//--------------- GenericHWInterface -------------------
class GenericHWInterface : public hardware_interface::RobotHW {
protected:
	enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

	ros::NodeHandle nh_;

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
	std::map<std::string, int> joint_names_;
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

	virtual std::string getURDF(ros::NodeHandle &nh, std::string param_name) const {
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



	void registerJointLimits(const std::string& joint_name,
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


public:
	GenericHWInterface(ros::NodeHandle &nh): nh_(nh),
		use_rosparam_joint_limits_(false),
		use_soft_limits_if_available_(false) {
	}

	virtual ~GenericHWInterface() {}

	virtual bool init() {
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
		num_joints_ = transmissions.size();
		//joint_names_.resize(num_joints_);
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

		ROS_DEBUG("GenericHWInterface start init joints(%lu)....", num_joints_);
		// Initialize interfaces for each joint
		for(unsigned int j=0; j < num_joints_; j++) {
			ROS_DEBUG("GenericHWInterface init joints(%u)....", j);
			// Check that this transmission has one joint
			if(transmissions[j].joints_.size() == 0) {
				ROS_WARN_STREAM("Transmission " << transmissions[j].name_ << " has no associated joints.");
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
				ROS_WARN_STREAM( "The <hardware_interface> element of tranmission " << transmissions[j].name_ 
								<< " should be nested inside the <joint> element, not <actuator>. " 
								<< "The transmission will be properly loaded, but please update " 
								<< "your robot model to remain compatible with future versions of the plugin.");
			}
			if (joint_interfaces.empty()) {
				ROS_WARN_STREAM( "Joint " << transmissions[j].joints_[0].name_ << " of transmission " 
								<< transmissions[j].name_ << " does not specify any hardware interface. " 
								<< "Not adding it to the robot hardware simulation.");
				continue;
			} else if (joint_interfaces.size() > 1) {
				ROS_WARN_STREAM( "Joint " << transmissions[j].joints_[0].name_ << " of transmission " 
								<< transmissions[j].name_ << " specifies multiple hardware interfaces. " 
								<< "Currently the default robot hardware simulation interface only supports one."
								<< "Using the first entry");
				continue;
			}

			// Add data from transmission
			std::string joint_name = transmissions[j].joints_[0].name_;
			joint_names_[ joint_name ]= j;
			joint_position_[j] = 1.0;
			joint_velocity_[j] = 0.0;
			joint_effort_[j] = 1.0;  // N/m for continuous joints
			joint_effort_command_[j] = 0.0;
			joint_position_command_[j] = 0.0;
			joint_velocity_command_[j] = 0.0;

			const std::string& hardware_interface = joint_interfaces.front();
			
			ROS_DEBUG_STREAM("Loading joint '" << joint_name << "' of type '" << hardware_interface << "'");

			// Create joint state interface for all joints
			joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
			        joint_name, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

			// Decide what kind of command interface this actuator/joint has
			hardware_interface::JointHandle joint_handle;
			if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface") {
				// Create effort joint interface
				joint_control_methods_[j] = EFFORT;
				joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_name), &joint_effort_command_[j]);
				effort_joint_interface_.registerHandle(joint_handle);
			} else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface") {
				// Create position joint interface
				joint_control_methods_[j] = POSITION;
				joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_name), &joint_position_command_[j]);
				position_joint_interface_.registerHandle(joint_handle);
			} else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface") {
				// Create velocity joint interface
				joint_control_methods_[j] = VELOCITY;
				joint_handle = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_name), &joint_velocity_command_[j]);
				velocity_joint_interface_.registerHandle(joint_handle);
			} else {
				ROS_FATAL_STREAM("No matching hardware interface found for '"
				                 << hardware_interface << "' while loading interfaces for " << joint_name );
				return false;
			}

			if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
				ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_name << "'.");
			}

			const ros::NodeHandle joint_limit_nh(nh_);
			registerJointLimits(joint_name, joint_handle, joint_control_methods_[j],
			                    joint_limit_nh, &urdf_model,
			                    &joint_types_[j], &joint_position_lower_limits_[j], &joint_position_upper_limits_[j],
			                    &joint_velocity_limits_[j], &joint_effort_limits_[j]);

			// Initialize the PID controller
			if (joint_control_methods_[j] != EFFORT) {
				//If no PID gain values are found, use joint->SetAngle() or joint->SetParam("vel") to control the joint.
				const ros::NodeHandle nh(nh_, "/ros_control/pid_gains/" + joint_name);
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
		ROS_DEBUG("GenericHWInterface register joints over.");
		return true;
	}

	virtual void cleanup() { }

	virtual bool start() = 0;

	virtual void stop() = 0;

	virtual void read(const ros::Time& time, const ros::Duration &period) = 0;

	virtual void write(const ros::Time& time, const ros::Duration &period) = 0;

	virtual void printState() {
		ROS_INFO_STREAM_THROTTLE(1, std::endl<< printStateHelper());
	}

	std::string printStateHelper() {
		std::stringstream ss;
		std::cout.precision(15);

		for (std::map<std::string,int>::iterator it=joint_names_.begin(); it!=joint_names_.end(); ++it) {
			ss << "j" << it->second << "(" << it->first << "): " << std::fixed << joint_position_[it->second] << "\t ";
			ss << std::fixed << joint_velocity_[it->second] << "\t ";
			ss << std::fixed << joint_effort_[it->second] << std::endl;
		}
		return ss.str();
	}

	std::string printCommandHelper() {
		std::stringstream ss;
		std::cout.precision(15);
		ss << "    position     velocity         effort  \n";
		for (std::map<std::string,int>::iterator it=joint_names_.begin(); it!=joint_names_.end(); ++it) {
			ss << "j" << it->second << "(" << it->first << "): " << std::fixed << joint_position_command_[it->second] << "\t ";
			ss << std::fixed << joint_velocity_command_[it->second] << "\t ";
			ss << std::fixed << joint_effort_command_[it->second] << std::endl;
		}
		return ss.str();
	}
};


//--------------- GenericHWControlLoop -------------------
static const double BILLION = 1000000000.0;

class GenericHWControlLoop {
protected:
	ros::NodeHandle nh_;

	ros::Duration desired_update_freq_;
	double cycle_time_error_threshold_;

	// Timing
	double loop_hz_;
	ros::Timer non_realtime_loop_;

	ros::Duration elapsed_time_;
	ros::Time last_;
	ros::Time now_;


	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	boost::shared_ptr<hw_control_template::GenericHWInterface> hardware_interface_;

public:
	GenericHWControlLoop(ros::NodeHandle& nh, boost::shared_ptr<hw_control_template::GenericHWInterface> hardware_interface)
		: nh_(nh), hardware_interface_(hardware_interface) {
		// Create the controller manager
		controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

		// Load rosparams
		ros::NodeHandle rpsnh(nh, "generic_hw_control_loop");
		std::size_t error = 0;
		if (!rpsnh.getParam("loop_hz",loop_hz_)) {
			ROS_FATAL("No rosparam: loop_hz" );
		}
		if (!rpsnh.getParam("cycle_time_error_threshold",cycle_time_error_threshold_)) {
			ROS_FATAL("No rosparam: cycle_time_error_threshold" );
		}

		// Get current time for use with first update
		struct timespec ts;
		if (!clock_gettime(CLOCK_REALTIME, &ts)) {
			last_.sec = ts.tv_sec;
			last_.nsec = ts.tv_nsec;
		}

		// Start timer
		desired_update_freq_ = ros::Duration(1 / loop_hz_);
		ROS_DEBUG_STREAM("GenericHWControlLoop start...");
		non_realtime_loop_ = nh_.createTimer(desired_update_freq_, &GenericHWControlLoop::update, this);
	}

	void update(const ros::TimerEvent& e) {
		struct timespec ts;
		if (!clock_gettime(CLOCK_REALTIME, &ts)) {
			now_.sec = ts.tv_sec;
			now_.nsec = ts.tv_nsec;
			elapsed_time_ = now_ - last_;
			last_ = now_;
		} else {
			ROS_FATAL("Failed to poll realtime clock!");
		}

		// Error check cycle time
		const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
		if (cycle_time_error > cycle_time_error_threshold_) {
			ROS_WARN_STREAM( "Cycle time exceeded error threshold by: "
			                 << cycle_time_error << ", cycle time: " << elapsed_time_
			                 << ", threshold: " << cycle_time_error_threshold_);
		}

		// Input
		hardware_interface_->read(now_, elapsed_time_);

		// Control
		controller_manager_->update(now_, elapsed_time_);

		// Output
		hardware_interface_->write(now_, elapsed_time_);
	}

};

}


namespace c2_hw {

double clamp(const double val, const double min_val, const double max_val) {
	return std::min(std::max(val, min_val), max_val);
}


class TestWheelHWInterface : public hw_control_template::GenericHWInterface {
protected:
	class StateBox {
	public:
		double *state_;
		double tmp_;

		StateBox():tmp_(0) {}

		void state_cb(const std_msgs::Float32ConstPtr& msg) {
			tmp_ = msg->data;
		}

		void assign() {
			*state_ = tmp_;
		}
	};

	std::vector<std::string> test_joint_;
	std::vector<int> test_joint_no_;
	std::vector<boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray> > > cmd_pub_;
	std::vector<StateBox> state_boxs_;
	std::vector<ros::Subscriber> sub_state_;
	
	
	ros::NodeHandle hw_nh_;

public:
	TestWheelHWInterface(ros::NodeHandle& nh): hw_control_template::GenericHWInterface(nh), hw_nh_(nh, "hardware_interface") {}

	virtual bool init() {
		hw_control_template::GenericHWInterface::init();

		//-------------- configure ----------------------
		ROS_DEBUG_STREAM("configure in [" << nh_.getNamespace() << "] ...");

		if (!hw_nh_.getParam("joints",test_joint_) ) {
			ROS_WARN_STREAM("Could not found ros param '"<< hw_nh_.getNamespace() << "/joints'." );
			return false;
		}

		int test_num = test_joint_.size();
		test_joint_no_.resize(test_num);
		cmd_pub_.resize(test_num);
		state_boxs_.resize(test_num);
		sub_state_.resize(test_num);
		for(int j=0; j < test_num; j++) {
			//找出变量
			std::map<std::string, int>::iterator it = joint_names_.find( test_joint_[j] );
			if ( it == joint_names_.end() ) {
				ROS_WARN_STREAM("can't find joint: " << test_joint_[j] << "!");
				continue;
			}
			test_joint_no_[j] = it->second;
			ROS_INFO_STREAM("find joint: " << test_joint_[j] << "...");

			//连接硬件接口
			std::string cmd_topic;
			if (!hw_nh_.getParam(test_joint_[j]+"/cmd_topic", cmd_topic) ) {
				ROS_WARN_STREAM("Could not found ros param '"<< hw_nh_.getNamespace()<< "/" << test_joint_[j] << "/cmd_topic'." );
			} else {
				cmd_pub_[j].reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh_, cmd_topic, 20));
				cmd_pub_[j]->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
				cmd_pub_[j]->msg_.layout.dim[0].size = 2;
				cmd_pub_[j]->msg_.layout.dim[0].stride = 1;
				cmd_pub_[j]->msg_.layout.dim[0].label = "speed";
				cmd_pub_[j]->msg_.data.resize(2);
				cmd_pub_[j]->msg_.data[0] = 0;
				cmd_pub_[j]->msg_.data[1] = 0;
				ROS_DEBUG_STREAM("publish: ' " << nh_.getNamespace()<< "/" << cmd_topic << "." );
			}

			std::string state_topic;
			if (!hw_nh_.getParam(test_joint_[j]+"/state_topic", state_topic) ) {
				ROS_WARN_STREAM("Could not found ros param '"<< hw_nh_.getNamespace()<< "/" << test_joint_[j] << "/state_topic'." );
			} else {
				if ( joint_control_methods_[it->second] == EFFORT ) {
					state_boxs_[j].state_ = &joint_effort_[j];
				} else if ( joint_control_methods_[it->second] == POSITION ) {
					state_boxs_[j].state_ = &joint_position_[j];
				} else if ( joint_control_methods_[it->second] == VELOCITY ) {
					state_boxs_[j].state_ = &joint_velocity_[j];
				}
				sub_state_[j] = nh_.subscribe<std_msgs::Float32>( state_topic, 10, &StateBox::state_cb, &(state_boxs_[j]));
				ROS_DEBUG_STREAM("subscribe: ' " << nh_.getNamespace()<< "/" <<state_topic << "." );
			}
		}
		
		
		ROS_DEBUG_STREAM("TestWheelHWInterface inited!");
	}

	virtual bool start() {

	}

	virtual void stop() {

	}

	virtual void read(const ros::Time& time, const ros::Duration &period) {
		//ROS_DEBUG_STREAM("TestWheelHWInterface reading...");

		for(unsigned int j=0; j < test_joint_.size(); j++) {
			state_boxs_[j].assign();
		}
	}

	virtual void write(const ros::Time& time, const ros::Duration &period) {
		//ROS_DEBUG_STREAM("TestWheelHWInterface writing...");

		pos_jnt_sat_interface_.enforceLimits(period);
		vel_jnt_sat_interface_.enforceLimits(period);
		eff_jnt_sat_interface_.enforceLimits(period);

		pos_jnt_soft_limits_.enforceLimits(period);
		vel_jnt_soft_limits_.enforceLimits(period);
		eff_jnt_soft_limits_.enforceLimits(period);

		for(unsigned int j=0; j < test_joint_.size(); j++) {
			int i = test_joint_no_[j];
			double cmd;
			if( joint_control_methods_[i] == EFFORT ) {
				cmd = joint_effort_command_[i];
			} else if ( joint_control_methods_[i] == POSITION) {
				cmd = joint_position_command_[i];
			} else if ( joint_control_methods_[i] == VELOCITY) {
				cmd = joint_velocity_command_[i];
			}
			if (cmd_pub_[j]->trylock()) {
				cmd_pub_[j]->msg_.data[0] = (cmd>0)?(int)floor(cmd):(int)ceil(cmd);
				cmd_pub_[j]->msg_.data[1] = cmd;
				cmd_pub_[j]->unlockAndPublish();
			}
		}
	}

};

}


//********** Main ************************
int main( int argc, char** argv ) {
	std::cout << "start Test ...";
	ROS_INFO_NAMED("test_wheel","test_wheel MAIN ...");

	ros::init(argc, argv, "test_wheel");
	ros::NodeHandle nh;

	// NOTE: We run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Create the hardware interface specific to your robot
	boost::shared_ptr<c2_hw::TestWheelHWInterface> robot(new c2_hw::TestWheelHWInterface(nh));
	robot->init();
	robot->start();

	// Start the control loop
	hw_control_template::GenericHWControlLoop control_loop(nh, robot);

	// Wait until shutdown signal recieved
	ros::waitForShutdown();
	return 0;
}
