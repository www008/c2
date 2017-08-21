#ifndef WHEEL_DRIVER_H
#define WHEEL_DRIVER_H

#include <iostream>

#include <ros/ros.h>

#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

#include "c2_hw/c2_hw.h"

namespace c2_hw {

class WheelDriver {
protected:
	ros::NodeHandle& nh_;
	
	boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Int8MultiArray> > vel_cmd_pub_;
	ros::Subscriber vel_state_sub_;

	double* p_vel_state_[2];
	double* p_vel_cmd_[2];
	
	bool state_update_;
	double vel_state_[2];
public:

	WheelDriver(ros::NodeHandle& nh): nh_(nh){
	}
	
	bool configure(c2_hw::C2HW& hw) {
		ROS_DEBUG_STREAM("WheelDriver configure in [" << nh_.getNamespace() << "] ...");

		// Get joint names from the parameter server
		std::string left_wheel_name, right_wheel_name;
		if (!nh_.getParam("left_wheel",left_wheel_name) || !nh_.getParam("right_wheel",right_wheel_name) ) {
			ROS_WARN_STREAM("Could not found 'left_wheel' or 'right_wheel' in ros param.");
			return false;
		}
		
		ROS_DEBUG_STREAM("WheelDriver found wheel joint name:[ " << left_wheel_name << " , "<< right_wheel_name << "]" );
		
		int f=0;
		for(int j=0; j< hw.joint_names_.size() && f<2; j++) {
			ROS_DEBUG_STREAM("WheelDriver hw joint: " << hw.joint_names_[j] );
			if (hw.joint_names_[j] == left_wheel_name) {
				p_vel_state_[0] = &hw.joint_velocity_[j];
				p_vel_cmd_[0] = &hw.joint_velocity_command_[j];
				f++;
				ROS_DEBUG("WheelDriver left wheel joint found!");
			}else if (hw.joint_names_[j] == right_wheel_name) {
				p_vel_state_[1] = &hw.joint_velocity_[j];
				p_vel_cmd_[1] = &hw.joint_velocity_command_[j];
				f++;
				ROS_DEBUG("WheelDriver right wheel joint found!");
			}
		}
		if(f!=2) {
			ROS_WARN_STREAM("Could not found joint: '" << left_wheel_name <<"' or '" << right_wheel_name <<"' in urdf.");
			return false;
		}
		return true;
	}

	void init() {
		vel_cmd_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int8MultiArray>(nh_, "hardware/wheels_cmd", 100));
		vel_cmd_pub_->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		vel_cmd_pub_->msg_.layout.dim[0].label = "diff_speed";
		vel_cmd_pub_->msg_.layout.dim[0].size = 2;
		vel_cmd_pub_->msg_.layout.dim[0].stride = 1;
		vel_cmd_pub_->msg_.layout.data_offset = 0;
		vel_cmd_pub_->msg_.data.resize(2);
		vel_cmd_pub_->msg_.data[0] = 0;
		vel_cmd_pub_->msg_.data[1] = 0;
		
		state_update_ = false;
		vel_state_sub_ = nh_.subscribe("hardware/wheels_state", 10, &WheelDriver::stateCB, this);
		ROS_INFO("WheelDriver init over.");
	}

	void cleanup() {
	}

	inline void setZero() {
		setSpeed( 0, 0);
	}

	void readState() {
		if (state_update_) {
			*p_vel_state_[0] = vel_state_[0];
			*p_vel_state_[1] = vel_state_[1];
			state_update_ = false;
		}
	}

	void writeCmd() {
		setSpeed(*p_vel_cmd_[0],*p_vel_cmd_[1]);
	}

protected:
	void setSpeed(const double left_vel, const double right_vel ) {
		int8_t left = toStep(left_vel);
		int8_t right = toStep(right_vel);
		if (vel_cmd_pub_->trylock()) {
			vel_cmd_pub_->msg_.data[0] = left;
			vel_cmd_pub_->msg_.data[1] = right;
			vel_cmd_pub_->unlockAndPublish();
		}
	}
	
	inline int8_t toStep(const double vel) {
		return (int8_t)(ceil(vel));
	}
	
	void stateCB(const std_msgs::Float32MultiArray& msg) {
		state_update_ = true;
		vel_state_[0] = msg.data[0];
		vel_state_[1] = msg.data[1];
	}
};

}
#endif
