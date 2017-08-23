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


class Evaluate {
	typedef struct item{
		double v;
		struct timespec t;
		item( struct timespec& t, double& v) {
			this->v = v;
			this->t = t;
		}
	} Item;
	
	queue<Item> q_;
	int max_size_;
	
public:
	double target_;
	
	Evaluate(int max): max_size_(max){}
	
	void push(struct timespec& t, double v){
		if(q_.size() < max_size_) {
			q_.push(Item(t,v));
		}
	}
	
	void clear(){
		Tr = Tp = Ts = 0;
		Sigma = 0;
		Mean = 0;
		Var = 0;
	}
	
	//动态跟随指标
	float Tr; 		//上升时间(rise time)，从0起第一次上升到稳态值C所需的时间。系统响应速度的一种度量。
	float Tp; 		//峰值时间(peak time), 指响应超过其稳态值达到第一个峰值所需的时间。系统响应速度的一种度量。
	float Sigma; 	//超调量(overshoot)：σ %， = (最大输出量 - 稳态值) / 稳态值 * 100%。评价系统的阻尼程度。
	float Ts;		//调节时间(setting time), 指响应到达并保持在稳态值±5%(或±2%)内所需的最短时间。评价系统响应速度和阻尼程度的综合指标
	float Mean; 	//均值
	float Var;		//方差
	
	void compute() {
		int state=0;
		double v_max = 0;
		double v_pmax = 0;
		bool climb = true;
		struct timespec start_time = q_.front().t;
		struct timespec tr,tp, ts;
		int s_cnt=0;
		double sum=0;
		double var=0;
		float size = q_.size();
		while( !q_.empty() ) {
			Item item = q_.front();
			q_.pop();
			if (state == 0 && abs(target_) <= abs(item.v) ) {
				tr = item.t;
				
				state = 1;
				ts = item.t;
			}
			if(v_max <= abs(item.v) ) {
				v_max = abs(item.v);
				if ( climb  ) {
					v_pmax = v_max;
					tp = item.t;
				}
			} else {
				if( climb && v_max >= abs(target_) ) {
					climb = false;
				}
			}
			if(state ==1) {
				if( abs( (target_ - item.v) / target_ ) < 0.05 ) {
					s_cnt++;
				}else {
					s_cnt =0;
					ts = item.t;
				}
				if(s_cnt > 5 ) {
					state=2;
				}
			}
			if( state > 0 ) {
				sum += item.v;
				var += ( item.v * item.v ); 
			}
		}
		Sigma = (v_max - abs(target_)) / abs(target_) *100;
		Tr = (tr.tv_sec - start_time.tv_sec) + (tr.tv_nsec - start_time.tv_nsec)/1000000.0f ;
		Tp = (tp.tv_sec - start_time.tv_sec) + (tp.tv_nsec - start_time.tv_nsec)/1000000.0f ;
		Ts = (ts.tv_sec - start_time.tv_sec) + (ts.tv_nsec - start_time.tv_nsec)/1000000.0f ;
		Mean = sum / size;
		Var = var/size - Mean*Mean;
		
		cout << "-->target=" << target_ << ", mean=" << Mean << ", var=" << Var 
			<< " [rise time]=" << Tr << "s, [peak time]=" << Tp << "s,"
			<< " [setting time]=" << Ts << "s, [overshoot]=" << Sigma << "%" << endl;
	}
	
};

typedef struct box_cb{
	Evaluate& ev;
	
	box_cb(Evaluate& v): ev(v){}
	
	void cb(const std_msgs::Float32::ConstPtr& msg) {
		if (startCN) {
			struct timespec current_time = {0,0};
			clock_gettime(CLOCK_REALTIME, &current_time);
			ev.push(current_time ,msg->data);
		}
	}
} BoxCb;



int main(int argc, char** argv) {
	ros::init(argc, argv, "tool_wheel_pid");
	ros::NodeHandle nh("tool_wheel_pid");
	
	std_msgs::Float32MultiArray pid_msg;
	ros::Publisher l_pid_pub = nh.advertise<std_msgs::Float32MultiArray>("/lwheel_pid", 10);
	ros::Publisher r_pid_pub = nh.advertise<std_msgs::Float32MultiArray>("/rwheel_pid", 10);
	pid_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	pid_msg.layout.dim[0].label = "c";
	pid_msg.layout.dim[0].size = 6;
	pid_msg.layout.dim[0].stride = 6;
	pid_msg.data.resize(6);

	XmlRpc::XmlRpcValue pid;
	if (nh.getParam("l_pid", pid) && pid.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		pid_msg.data[0] = (double)pid["p"];
		pid_msg.data[1] = (double)pid["i"];
		pid_msg.data[2] = (double)pid["d"];
		pid_msg.data[3] = (double)pid["v_max"];
		pid_msg.data[4] = (double)pid["v_min"];
		pid_msg.data[5] = (double)pid["i_err_max"];
		l_pid_pub.publish(pid_msg);
		
		ROS_INFO_STREAM( "l_pid:" << pid_msg.data[0] << ", " << pid_msg.data[1] << ", " << pid_msg.data[2] << ", "
				<< pid_msg.data[3] << ", " << pid_msg.data[4] << ", " << pid_msg.data[5] << "." );
	}
	if (nh.getParam("r_pid", pid) && pid.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		pid_msg.data[0] = (double)pid["p"];
		pid_msg.data[1] = (double)pid["i"];
		pid_msg.data[2] = (double)pid["d"];
		pid_msg.data[3] = (double)pid["v_max"];
		pid_msg.data[4] = (double)pid["v_min"];
		pid_msg.data[5] = (double)pid["i_err_max"];
		r_pid_pub.publish(pid_msg);
		
		ROS_INFO_STREAM( "r_pid:" << pid_msg.data[0] << ", " << pid_msg.data[1] << ", " << pid_msg.data[2] << ", "
				<< pid_msg.data[3] << ", " << pid_msg.data[4] << ", " << pid_msg.data[5] << "." );
	}
	ros::spinOnce();
	
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
			double left_vel = plan_list[i]["l"];
			double right_vel = plan_list[i]["r"];
			plan.push(PlanStep( lasting, left_vel, right_vel));
			ROS_DEBUG_STREAM( "Step "<< i << ": t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
        }
	}else if (plan_list.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
		int lasting = (int)(plan_list["t"]);
		double left_vel = plan_list["l"];
		double right_vel = plan_list["r"];
		ROS_DEBUG_STREAM( "Step: t=" << lasting << ", left=" << left_vel << ", right=" << right_vel << "." );
		plan.push(PlanStep( lasting, left_vel, right_vel));
	}
	ROS_INFO("planstep:%u", (unsigned int)(plan.size()) );
	
	//-----------------
	ros::Rate loop_rate(10);
	
	ros::Publisher l_pub = nh.advertise<std_msgs::Float32MultiArray>("/lwheel_vtarget", 10);
	ros::Publisher r_pub = nh.advertise<std_msgs::Float32MultiArray>("/rwheel_vtarget", 10);
	
	Evaluate ev_l(50);
	Evaluate ev_r(50);
	BoxCb lcb(ev_l);
	BoxCb rcb(ev_r);

	ros::Subscriber l_sub = nh.subscribe("/lwheel_state",10, &BoxCb::cb, &lcb);
	ros::Subscriber r_sub = nh.subscribe("/rwheel_state",10, &BoxCb::cb, &rcb);

	std_msgs::Float32MultiArray msg;

	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].label = "step";
	msg.layout.dim[0].size = 2;
	msg.layout.dim[0].stride = 2;
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[1].label = "vel";
	msg.layout.dim[1].size = 1;
	msg.layout.dim[1].stride = 1;
	msg.data.resize(2);
	msg.data[0] = 0;
	msg.data[1] = 0;

	//-----------------
	while(!plan.empty()) {
		PlanStep step = plan.front();
		ROS_DEBUG_STREAM( "Exec: t=" << step.lasting << ", left=" << step.left_step << ", right=" << step.right_step << "." );
		plan.pop();
		struct timespec current_time = {0,0};
		clock_gettime(CLOCK_REALTIME, &current_time);
		struct timespec stop_time = {current_time.tv_sec + step.lasting, 0};
		
		ev_l.clear();
		ev_l.target_ = step.left_step;
		ev_r.clear();
		ev_r.target_ = step.right_step;

		startCN = true;
		while (ros::ok()) {
			clock_gettime(CLOCK_REALTIME, &current_time);
			if(current_time.tv_sec > stop_time.tv_sec ) {
				break;
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
cout << "compute vel" << endl;
		//compute vel
		startCN = false;
		ev_l.compute();
		ev_r.compute();
		
		//stop and wait 4s
		msg.data[0] = 0;
		msg.data[1] = 0;
		l_pub.publish(msg);
		msg.data[0] = 0;
		msg.data[1] = 0;
		r_pub.publish(msg);

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
cout << "stop over!" << endl;
	}
	
	return(0);
}
