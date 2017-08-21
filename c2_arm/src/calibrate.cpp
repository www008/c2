#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include <string>
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

typedef struct plan_step{
	int lasting;
	vector<string> joint_names;
	vector<double> joint_pos;
	
	/*struct plan_step& operator=(const struct plan_step& obj){
		this->lasting = obj.lasting;
		this->joint_names = obj.joint_names;
		this->joint_pos = obj.joint_pos;
		return *this;
	}*/
} PlanStep;

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibrate");
	ros::NodeHandle nh("calibrate_arm");

	//get plan
	std::queue<PlanStep> plan;
	XmlRpc::XmlRpcValue plan_list;
	if (!nh.getParam("plan", plan_list)) {
		ROS_ERROR( "Couldn't retrieve plan param.");
		return -2;
	}
	
	if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		for (int i = 0; i < plan_list.size(); ++i) {
			PlanStep step;
			ROS_DEBUG_STREAM( "Step "<< i << ":");
			if (plan_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
				ROS_ERROR_STREAM( "plan isn't vaild struct line#" << i << ".");
				return -3;
			}
			step.lasting = (int)(plan_list[i]["t"]);
			
			if (plan_list[i]["joints"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
				XmlRpc::XmlRpcValue& joint_names = plan_list[i]["joints"];
				for (int j = 0; j < joint_names.size(); ++j) {
					step.joint_names.push_back( static_cast<string>(joint_names[j]));
					ROS_DEBUG_STREAM( "read joint:" << static_cast<string>(joint_names[j]) );
				}
			}
			if (plan_list[i]["pos"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
				XmlRpc::XmlRpcValue& joint_pos = plan_list[i]["pos"];
				for (int j = 0; j < joint_pos.size(); ++j) {
					if(joint_pos[j].getType() == XmlRpc::XmlRpcValue::TypeDouble) 
						cout << "double is ok" << endl;
					else
						cout << "double is error" << endl;
					
					step.joint_pos.push_back(static_cast<double>(joint_pos[j]));
					ROS_DEBUG_STREAM( "read joint pos:"<< static_cast<double>(joint_pos[j]) );
				}
			}
			plan.push(step);
        }
		ROS_INFO("planstep:%u", (unsigned int)(plan.size()) );
	}else {
		ROS_ERROR("Plan is error!!!");
		return -1;
	}
	

	//publish msg
	ros::Rate loop_rate(10);
	
	ros::Publisher arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_pos", 10);
	trajectory_msgs::JointTrajectory msg;
	msg.header.frame_id = "arm_base";
	msg.joint_names.resize(0);
	msg.points.resize(0);
	while(!plan.empty()) {
		PlanStep step = plan.front();
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
			
			msg.header.stamp.sec = current_time.tv_sec;
			msg.header.stamp.nsec = current_time.tv_nsec;
			msg.joint_names.resize(0);
			msg.points.resize(0);
			msg.joint_names = step.joint_names;
			for (int i=0; i<step.joint_pos.size(); i++) {
				msg.points.push_back(trajectory_msgs::JointTrajectoryPoint());
				msg.points[i].positions.push_back(step.joint_pos[i]);
			}
			
			arm_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	return 0;
}
