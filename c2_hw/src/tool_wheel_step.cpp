#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "c2_hw/dqueue.h"

using namespace std;

typedef struct plan_step{
	int lasting;
	double left_step;
	double right_step;
	
	dqueue<double, 20> l_vel;
	dqueue<double, 20> r_vel;
	
	plan_step(int l, double l_step, double r_step): lasting(l),left_step(l_step),right_step(r_step)  {}
} PlanStep;

bool startCN;

typedef struct box_cb{
	dqueue<double, 20>& vel;
	
	box_cb(dqueue<double, 20>& v): vel(v){}
	
	void cb(const std_msgs::Float32::ConstPtr& msg) {
		if (startCN)
			vel.push(msg->data);
	}
} BoxCb;

int main(int argc, char** argv) {
	ros::init(argc, argv, "tool_wheel_step");
	ros::NodeHandle nh("tool_wheel_step");
	//get plan
	std::queue<PlanStep> plan;
	XmlRpc::XmlRpcValue plan_list;
	if (!nh.getParam("plan", plan_list)) {
		ROS_ERROR( "Couldn't retrieve plan param.");
		return -2;
	}
	if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		for (int i = 0; i < plan_list.size(); ++i) {
			cout << "get "<< i << endl;
			if (plan_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
				ROS_ERROR_STREAM( "plan isn't vaild struct line#" << i << ".");
				cout << "init -3" << endl;
				return -3;
			}
			int lasting = (int)(plan_list[i]["t"]);
			double left_vel = (int)(plan_list[i]["l"]);
			double right_vel = (int)(plan_list[i]["r"]);
			plan.push(PlanStep( lasting, left_vel, right_vel));
			ROS_DEBUG_STREAM( "Step "<< i << ": t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
        }
	}else if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		int lasting = (int)(plan_list["t"]);
		double left_vel = (int)(plan_list["l"]);
		double right_vel = (int)(plan_list["r"]);
		ROS_DEBUG_STREAM( "Step: t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
		plan.push(PlanStep( lasting, left_vel, right_vel));
	}
	ROS_INFO("planstep:%u", (unsigned int)(plan.size()) );
	
	//-----------------
	std::queue<double> l_steps, l_vels;
	std::queue<double> r_steps, r_vels;
	
	ros::Rate loop_rate(10);
	
	ros::Publisher l_pub = nh.advertise<std_msgs::Float32MultiArray>("/lwheel_vtarget", 10);
	ros::Publisher r_pub = nh.advertise<std_msgs::Float32MultiArray>("/rwheel_vtarget", 10);
	
	dqueue<double, 20> q_l,q_r;
	BoxCb lcb(q_l);
	BoxCb rcb(q_r);
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
	
	//-----------------
	startCN = false;
	while(!plan.empty()) {
		PlanStep step = plan.front();
		ROS_DEBUG_STREAM( "Exec: t=" << step.lasting << ", left=" << step.left_step << ", right=" << step.right_step << "." );
		plan.pop();
		struct timespec current_time = {0,0};
		clock_gettime(CLOCK_REALTIME, &current_time);
		struct timespec stop_time = {current_time.tv_sec + step.lasting, 0};
		struct timespec start_time = {current_time.tv_sec + 4, 0};
		
		q_l.clear();
		q_r.clear();
		
		while (ros::ok()) {
			clock_gettime(CLOCK_REALTIME, &current_time);
			if(current_time.tv_sec > stop_time.tv_sec ) {
				break;
			}
			if(current_time.tv_sec > start_time.tv_sec ) {
				startCN = true;
			}
			
			msg.data[0] = step.left_step;
			msg.data[1] = 0;
			l_pub.publish(msg);
			msg.data[0] = step.right_step;
			msg.data[1] = 0;
			r_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}

		//compute vel
		startCN = false;
		double mean,var,dsize;
		mean = var = 0;
		dsize = q_l.size();
		for(dqueue<double, 20>::iterator it=q_l.begin(); it!=q_l.end(); it++ ){
			mean += *it;
			var += (*it) * (*it);
			cout << "~" << *it;
		}
		mean /= dsize;
		var = var/dsize - mean*mean;
		l_steps.push(step.left_step);
		l_vels.push(mean);
		ROS_INFO_STREAM( "left("<< dsize <<"): step=" << step.left_step << ", mean=" << mean << ", var=" << var );
		
		mean = var = 0;
		dsize = q_r.size();
		for(dqueue<double, 20>::iterator it=q_r.begin(); it!=q_r.end(); it++ ){
			mean += *it;
			var += (*it) * (*it);
			cout << "~" << *it;
		}
		mean /= dsize;
		var = var/dsize - mean*mean;
		r_steps.push(step.right_step);
		r_vels.push(mean);
		ROS_INFO_STREAM( "right("<< dsize <<"): step=" << step.right_step << ", mean=" << mean << ", var=" << var );
		
		//stop and wait 4s
		msg.data[0] = 0;
		msg.data[1] = 0;
		l_pub.publish(msg);
		msg.data[0] = 0;
		msg.data[1] = 0;
		r_pub.publish(msg);
		ROS_DEBUG_STREAM( "stop." );
		current_time = {0,0};
		clock_gettime(CLOCK_REALTIME, &current_time);
		stop_time = {current_time.tv_sec + 3, 0};
		while (ros::ok()) {
			clock_gettime(CLOCK_REALTIME, &current_time);
			if(current_time.tv_sec > stop_time.tv_sec ) {
				break;
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	//-------------
	cout << "left =[";
	while(!l_steps.empty()) {
		cout << l_steps.front() << ", " << l_vels.front() << "; ";
		l_steps.pop();
		l_vels.pop();
	}
	cout << "];" << endl;
	
	cout << "right =[";
	while(!r_steps.empty()) {
		cout << r_steps.front() << ", " << r_vels.front() << "; ";
		r_steps.pop();
		r_vels.pop();
	}
	cout << "];" << endl;
	
	return(0);
}
