#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <dynamic_reconfigure/server.h>
#include "c2_hw/CalibrateParamConfig.h"

class Calibrate {
private:
	ros::NodeHandle& nh_;

	struct Commands {
		double lin;
		double ang;
		double timeout;
		ros::Time stamp;

		Commands() : lin(0.0), ang(0.0), timeout(20), stamp(0.0) {}
	};
	realtime_tools::RealtimeBuffer<Commands> command_;
	Commands command_struct_;
	
	double default_cmd_timeout_;
	
	ros::Subscriber sub_cmd_vel_;
	
	boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray> > pub_lmotor_;
	boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray> > pub_rmotor_;
	
	dynamic_reconfigure::Server<c2_hw::CalibrateParamConfig> dyconf_server_;
public:
	Calibrate(ros::NodeHandle& nh): nh_(nh)  {}
	
	bool init(){
		nh_.param("default_cmd_timeout", default_cmd_timeout_, 20.0);
		
		sub_cmd_vel_ = nh_.subscribe("cmd_vel", 2, &Calibrate::cmdVelCB, this);

		pub_lmotor_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh_, "lwheel_vtarget", 10));
		pub_lmotor_->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		pub_lmotor_->msg_.layout.dim[0].label = "step";
		pub_lmotor_->msg_.layout.dim[0].size = 1;
		pub_lmotor_->msg_.layout.dim[0].stride = 2;
		pub_lmotor_->msg_.layout.dim[0].label = "vel";
		pub_lmotor_->msg_.layout.dim[0].size = 1;
		pub_lmotor_->msg_.layout.dim[0].stride = 1;
		//pub_lmotor_->msg_.layout.data_offset = 0;
		pub_lmotor_->msg_.data.resize(2);
		pub_lmotor_->msg_.data[0] = 0;
		pub_lmotor_->msg_.data[1] = 0;

		pub_rmotor_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh_, "rwheel_vtarget", 10));
		pub_rmotor_->msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		pub_rmotor_->msg_.layout.dim[0].label = "step";
		pub_rmotor_->msg_.layout.dim[0].size = 1;
		pub_rmotor_->msg_.layout.dim[0].stride = 2;
		pub_rmotor_->msg_.layout.dim[0].label = "vel";
		pub_rmotor_->msg_.layout.dim[0].size = 1;
		pub_rmotor_->msg_.layout.dim[0].stride = 1;
		//pub_rmotor_->msg_.layout.data_offset = 0;
		pub_rmotor_->msg_.data.resize(2);
		pub_rmotor_->msg_.data[0] = 0;
		pub_rmotor_->msg_.data[1] = 0;
		
		dynamic_reconfigure::Server<c2_hw::CalibrateParamConfig>::CallbackType f = boost::bind(&Calibrate::dyconfCB, this, _1, _2);
		dyconf_server_.setCallback(f);
		return true;
	}

	void update(const ros::TimerEvent& e) {
		ros::Time now = e.current_real;

		Commands curr_cmd = *(command_.readFromRT());
		const double dt = (now - curr_cmd.stamp).toSec();

		// Brake if cmd_vel has timeout:
		double vel_left, vel_right;
		if (dt > curr_cmd.timeout) {
			vel_left = vel_right = 0;
		}else { //wheel_separation_multiplier_
			vel_left  = curr_cmd.lin; 
			vel_right = curr_cmd.lin;
		}
		
		// Set wheels velocities:
		if (pub_lmotor_->trylock()) {
			pub_lmotor_->msg_.data[0] = vel_left;
			pub_lmotor_->unlockAndPublish();
		}
		if (pub_rmotor_->trylock()) {
			pub_rmotor_->msg_.data[0] = vel_right;
			pub_rmotor_->unlockAndPublish();
		}
	}
	
	void cmdVelCB(const geometry_msgs::Twist &command) {
		command_struct_.ang   = command.angular.z;
		command_struct_.lin   = command.linear.x;
		command_struct_.timeout = (command.linear.z!=0)?command.linear.z:default_cmd_timeout_;
		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT (command_struct_);
		ROS_DEBUG_STREAM("Added values to command. "
							   << "Ang: "   << command_struct_.ang << ", "
							   << "Lin: "   << command_struct_.lin << ", "
							   << "timeout: "<< command_struct_.timeout << ", "
							   << "Stamp: " << command_struct_.stamp);
	}
	
	void dyconfCB(c2_hw::CalibrateParamConfig &config, uint32_t level) {
		default_cmd_timeout_ = config.cmd_timeout;
		ROS_DEBUG_STREAM("Reconfigure -->" << "cmd_timeout: "   << config.cmd_timeout );
	}
	
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibrate_node");
	ros::NodeHandle nh("calibrate");

	Calibrate calib(nh);
	calib.init();

	double loop_hz = 20;
	nh.param("loop_hz", loop_hz);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration period = ros::Duration(1 / loop_hz);
	ROS_DEBUG_STREAM("Calibrate start...");
	ros::Timer loop = nh.createTimer(period, &Calibrate::update, &calib);

	ros::waitForShutdown();

	return(0);
}
