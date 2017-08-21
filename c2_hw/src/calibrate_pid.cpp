#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "c2_hw/pid.h"

typedef struct plan_step{
	int lasting;
	double left_vel;
	double right_vel;
	
	plan_step(int l, double l_vel, double r_vel): lasting(l),left_vel(l_vel),right_vel(r_vel)  {}
} PlanStep;


typedef struct box_cb{
	double& vel;
	
	box_cb(double& v): vel(v){}
	
	void cb(const std_msgs::Float32::ConstPtr& msg) {
		vel = msg->data;
	}
} BoxCb;

double toStep(double y, bool left) {
	double p[4][2]= { {0.000460621044978824, -0.230725259521080},
					  {0.000464563181859522,  0.222105248803710},
					  {0.000522881269809069, -0.149896696815505},
					  {0.000525115215061441,  0.157414667047491} };
	double x;
	if(left && y<0) {
		x = (y-p[0][1])/p[0][0];
	}else if(left && y>0) {
		x = (y-p[1][1])/p[1][0];
	}else if(!left && y<0) {
		x = (y-p[2][1])/p[2][0];
	}else if(!left && y>0) {
		x = (y-p[3][1])/p[3][0];
	}else{
		return 0;
	}
	return x;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibrate_pid");
	ros::NodeHandle nh("calibrate_pid");

	//get plan
	std::queue<PlanStep> plan;
	XmlRpc::XmlRpcValue plan_list;
	if (!nh.getParam("plan", plan_list)) {
		ROS_ERROR( "Couldn't retrieve plan param.");
		return -2;
	}
	
	if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		for (int i = 0; i < plan_list.size(); ++i) {
			if (plan_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
				ROS_ERROR_STREAM( "plan isn't vaild struct line#" << i << ".");
				return -3;
			}
			int lasting = (int)(plan_list[i]["t"]);
			double left_vel = (double)(plan_list[i]["l"]);
			double right_vel = (double)(plan_list[i]["r"]);
			plan.push(PlanStep( lasting, left_vel, right_vel));
			ROS_DEBUG_STREAM( "Step "<< i << ": t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
        }
	}else if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		int lasting = (int)(plan_list["t"]);
		double left_vel = (double)(plan_list["l"]);
		double right_vel = (double)(plan_list["r"]);
		ROS_DEBUG_STREAM( "Step: t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
		plan.push(PlanStep( lasting, left_vel, right_vel));
	}
	ROS_INFO("planstep:%u", (unsigned int)(plan.size()) );

	//-----------------
	ros::Rate loop_rate(10);
	
	ros::Publisher l_pub = nh.advertise<std_msgs::Float32MultiArray>("/lwheel_vtarget", 10);
	ros::Publisher r_pub = nh.advertise<std_msgs::Float32MultiArray>("/rwheel_vtarget", 10);
	
	double lvel_stat=0, rvel_stat=0;
	BoxCb lcb(lvel_stat);
	BoxCb rcb(rvel_stat);
	ros::Subscriber l_sub = nh.subscribe("/lwheel_state",10, &BoxCb::cb, &lcb);
	ros::Subscriber r_sub = nh.subscribe("/rwheel_state",10, &BoxCb::cb, &rcb);

	std_msgs::Float32MultiArray msg;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].label = "step";
	msg.layout.dim[0].size = 1;
	msg.layout.dim[0].stride = 2;
	msg.layout.dim[0].label = "vel";
	msg.layout.dim[0].size = 1;
	msg.layout.dim[0].stride = 1;
	msg.data.resize(2);
	msg.data[0] = 0;
	msg.data[1] = 0;

	//-------------
	c2_hw::Pid lwheel_pid;
	c2_hw::Pid rwheel_pid;

	if (!lwheel_pid.init(ros::NodeHandle(nh, "left_pid"))) {
		ROS_WARN("left_pid not exist!");
		return -3;
	}
	if (!rwheel_pid.init(ros::NodeHandle(nh, "right_pid"))) {
		ROS_WARN("right_pid not exist!");
		return -4;
	}
	
	//-----------------
	while(!plan.empty()) {
		PlanStep step = plan.front();
		ROS_DEBUG_STREAM( "Exec: t=" << step.lasting << ", left=" << step.left_vel << ", right=" << step.right_vel << "." );
		plan.pop();
		struct timespec current_time = {0,0};
		clock_gettime(CLOCK_REALTIME, &current_time);
		struct timespec stop_time = {current_time.tv_sec + step.lasting, 0};
		struct timespec last_time = current_time;
		while (ros::ok()) {
			clock_gettime(CLOCK_REALTIME, &current_time);
			if(current_time.tv_sec > stop_time.tv_sec ) {
				break;
			}
			ros::Duration elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0 );
			last_time = current_time;
			
			double lvel_error = step.left_vel - lvel_stat;
			double lvel_cmd = lwheel_pid.computeCommand(lvel_error, elapsed_time);
			double rvel_error = step.right_vel - rvel_stat;
			double rvel_cmd = rwheel_pid.computeCommand(rvel_error, elapsed_time);
			
			double l_step = toStep(lvel_cmd,true);
			double r_step = toStep(rvel_cmd,false);
			
			msg.data[0] = l_step;
			msg.data[1] = lvel_cmd;
			l_pub.publish(msg);
			msg.data[0] = r_step;
			msg.data[1] = rvel_cmd;
			r_pub.publish(msg);
			ROS_DEBUG_STREAM( "Publish left: step=" << l_step << ", cmd=" << lvel_cmd << ", error=" << lvel_error 
						<< "\t right: step=" << r_step << ", cmd=" << rvel_cmd << ", error=" << rvel_error << ".");
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	return(0);
}
