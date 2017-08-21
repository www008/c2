#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include <dynamic_reconfigure/server.h>
#include "c2_hw/CalibrateParamConfig.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibrate_linear");
	ros::NodeHandle nh;

	for(int i=0;i<argc; i++) {
		ROS_WARN("%d:%s", i, argv[i]);
	}
	
	if (argc < 4 ) {
		return -1;
	}
	bool isStep = (argv[1][0]=='P');
	int lastting = atoi(argv[2]);
	char *p;
	float l_vel, r_vel;
	if ( (p = strtok(argv[3],","))!=NULL ) {
		l_vel = r_vel = isStep?atoi(p):atof(p);
		if( (p=strtok(NULL,",")) !=NULL) {
			r_vel = isStep?atoi(p):atof(p);
		}
	}else{
		l_vel = r_vel = isStep?atoi(p):atof(p);
	}
	printf("lasting:%ds  l_vel:%f, r_vel:%f\r\n", lastting, l_vel, r_vel);
	
	ros::Rate loop_rate(10);
	
	ros::Publisher l_pub = nh.advertise<std_msgs::Float32MultiArray>("lwheel_vtarget", 10);
	ros::Publisher r_pub = nh.advertise<std_msgs::Float32MultiArray>("rwheel_vtarget", 10);

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
	
	struct timespec ts = {0,0};
	clock_gettime(CLOCK_REALTIME, &ts);
	struct timespec stopts = {ts.tv_sec+lastting, 0};
	
	while (ros::ok()) {
		clock_gettime(CLOCK_REALTIME, &ts);
		if(ts.tv_sec > stopts.tv_sec ) {
			break;
		}
		msg.data[isStep?0:1] = l_vel;
		l_pub.publish(msg);
		msg.data[isStep?0:1] = r_vel;
		r_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return(0);
}
