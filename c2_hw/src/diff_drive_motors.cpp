#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "c2_hw/odometry.h"



class DiffDriveMotors {
private:
	ros::NodeHandle& nh_;

	double wheel_separation_;
	double wheel_radius_;
	std::string base_frame_id_;

	double cmd_vel_timeout_;

	double state_lwheel_vel_ , state_rwheel_vel_;

	/// Hardware handles:
	boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32> > pub_lmotor_;
	boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float32> > pub_rmotor_;
	
	control_toolbox::Pid lwheel_pid_;
	control_toolbox::Pid rwheel_pid_;

	ros::Subscriber sub_lwheel_vel_;
	ros::Subscriber sub_rwheel_vel_;
	
	/// Velocity command related:
	struct Commands {
		double lin;
		double ang;
		ros::Time stamp;

		Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
	};
	realtime_tools::RealtimeBuffer<Commands> command_;
	Commands command_struct_;
	ros::Subscriber sub_cmd_vel_;

	/// Odometry related:
	ros::Duration publish_period_;
	ros::Time last_state_publish_time_;

	diff_drive_motors::Odometry odometry_;
	
	boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > pub_odom_;
	boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > pub_tf_odom_;
	
private:
	void cmdVelCallback(const geometry_msgs::Twist &command) {
		command_struct_.ang   = command.angular.z;
		command_struct_.lin   = command.linear.x;
		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT (command_struct_);
		ROS_DEBUG_STREAM("Added values to command. "
							   << "Ang: "   << command_struct_.ang << ", "
							   << "Lin: "   << command_struct_.lin << ", "
							   << "Stamp: " << command_struct_.stamp);
	}
	
	void leftVelCb(const std_msgs::Float32 &msg) {
		state_lwheel_vel_ = msg.data;
	}
	
	void rightVelCb(const std_msgs::Float32 &msg) {
		state_rwheel_vel_ = msg.data;
	}
	
public:
	DiffDriveMotors(ros::NodeHandle& nh): nh_(nh)
		, wheel_separation_(0.12)
		, wheel_radius_(0.032)
		, cmd_vel_timeout_(10)
		, base_frame_id_("base_footprint"){}
	
	bool init() {
		if(!nh_.getParam("wheel_separation", wheel_separation_)) {
			ROS_WARN_STREAM("wheel_separation  param is not set " );
		}
		
		if(!nh_.getParam("wheel_radius", wheel_radius_)) {
			ROS_WARN_STREAM("wheel_radius  param is not set " );
		}
		nh_.param("cmd_vel_timeout", cmd_vel_timeout_);
		nh_.param("base_frame_id", base_frame_id_);
		ROS_DEBUG_STREAM_NAMED("DiffDriveMotors","base_frame_id is:" << base_frame_id_ );
		
		//----- Twist -----  
		sub_cmd_vel_ = nh_.subscribe("cmd_vel", 10, &DiffDriveMotors::cmdVelCallback, this);

		pub_lmotor_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(nh_, "lwheel_vtarget", 50));
		pub_lmotor_->msg_.data = 0;
		pub_rmotor_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(nh_, "rwheel_vtarget", 50));
		pub_rmotor_->msg_.data = 0;
		
		//-------- Odometry related---
		double publish_rate;
		nh_.param("publish_rate", publish_rate, 2.0);
		ROS_INFO_STREAM("Odometry will be published at " << publish_rate << "Hz.");
		publish_period_ = ros::Duration(1.0 / publish_rate);
		
		sub_lwheel_vel_ = nh_.subscribe("lwheel_state",10, &DiffDriveMotors::leftVelCb, this);
		sub_rwheel_vel_ = nh_.subscribe("rwheel_state",10, &DiffDriveMotors::rightVelCb, this);
		
		pub_odom_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh_, "odom", 100));
		pub_odom_->msg_.header.frame_id = "odom";
		pub_odom_->msg_.child_frame_id = base_frame_id_;
		pub_odom_->msg_.pose.pose.position.z = 0;
		//pub_odom_->msg_.pose.covariance = boost::assign::list_of
		//                                  (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
		//                                  (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
		//                                  (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
		//                                  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
		//                                  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
		//                                  (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
		pub_odom_->msg_.twist.twist.linear.y  = 0;
		pub_odom_->msg_.twist.twist.linear.z  = 0;
		pub_odom_->msg_.twist.twist.angular.x = 0;
		pub_odom_->msg_.twist.twist.angular.y = 0;
		//pub_odom_->msg_.twist.covariance = boost::assign::list_of
		//                                   (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
		//                                   (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
		//                                   (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
		//                                   (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
		//                                   (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
		//                                   (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
		pub_tf_odom_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh_, "/tf", 100));
		pub_tf_odom_->msg_.transforms.resize(1);
		pub_tf_odom_->msg_.transforms[0].transform.translation.z = 0.0;
		pub_tf_odom_->msg_.transforms[0].child_frame_id = base_frame_id_;
		pub_tf_odom_->msg_.transforms[0].header.frame_id = "odom";
		
		//-------- starting -----------
		last_state_publish_time_ = ros::Time::now();
		
		odometry_.init(last_state_publish_time_);
		odometry_.setWheelParams(wheel_separation_ , wheel_radius_);
		
		//-------- PID ----------
		if (!lwheel_pid_.init(ros::NodeHandle(nh_, "left_wheel_pid")) 
			|| !rwheel_pid_.init(ros::NodeHandle(nh_, "right_wheel_pid")) )
			return false;
	}
	
	void update(const ros::TimerEvent& e) {
		ros::Time time= e.current_real;

		//---------- Odometry ---------
		// Estimate linear and angular velocity using joint information
		odometry_.update(state_lwheel_vel_, state_rwheel_vel_, time);

		// Publish odometry message
		if (last_state_publish_time_ + publish_period_ < time) {
			last_state_publish_time_ = time;
			// Compute and store orientation info
			const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

			// Populate odom message and publish
			if (pub_odom_->trylock()) {
				pub_odom_->msg_.header.stamp = time;
				pub_odom_->msg_.pose.pose.position.x = odometry_.getX();
				pub_odom_->msg_.pose.pose.position.y = odometry_.getY();
				pub_odom_->msg_.pose.pose.orientation = orientation;
				pub_odom_->msg_.twist.twist.linear.x  = odometry_.getLinear();
				pub_odom_->msg_.twist.twist.angular.z = odometry_.getAngular();
				pub_odom_->unlockAndPublish();
			}

			// Publish tf /odom frame
			if (pub_tf_odom_->trylock()) {
				geometry_msgs::TransformStamped& odom_frame = pub_tf_odom_->msg_.transforms[0];
				odom_frame.header.stamp = time;
				odom_frame.transform.translation.x = odometry_.getX();
				odom_frame.transform.translation.y = odometry_.getY();
				odom_frame.transform.rotation = orientation;
				pub_tf_odom_->unlockAndPublish();
			}
		}

		// MOVE ROBOT
		// Retreive current velocity command and time step:
		Commands curr_cmd = *(command_.readFromRT());
		const double dt = (time - curr_cmd.stamp).toSec();

		// Brake if cmd_vel has timeout:
		double vel_left, vel_right;
		if (dt > cmd_vel_timeout_) {
			vel_left = vel_right = 0;
		}else {
			vel_left  = curr_cmd.lin - (curr_cmd.ang * wheel_separation_ / 2.0);
			vel_right = curr_cmd.lin + (curr_cmd.ang * wheel_separation_ / 2.0);
		}

		//pid control 
		//double lwheel_error = vel_left - state_lwheel_vel_;
		//double lwheel_cmd = lwheel_pid_.computeCommand(lwheel_error, period);
		
		// Set wheels velocities:
		if (pub_lmotor_->trylock()) {
			pub_lmotor_->msg_.data = vel_left;
			pub_lmotor_->unlockAndPublish();
		}
		if (pub_rmotor_->trylock()) {
			pub_rmotor_->msg_.data = vel_right;
			pub_rmotor_->unlockAndPublish();
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv,"diff_drive_motors");
	ros::NodeHandle nh("diff_drive_motors");
	
	DiffDriveMotors obj(nh);
	obj.init();

	double loop_hz = 10;
	nh.getParam("loop_hz", loop_hz);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration period = ros::Duration(1 / loop_hz);
	ROS_DEBUG_STREAM("diff_drive_motors start...");
	ros::Timer loop = nh.createTimer(period, &DiffDriveMotors::update, &obj);

	ros::waitForShutdown();
	return 0;
}
