#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

#define KEYCODE_LV_D 0x5f
#define KEYCODE_LV_U 0x2b
#define KEYCODE_AV_D 0x2d
#define KEYCODE_AV_U 0x3d

#define KEYCODE_SP 0x20

int kfd = 0;
struct termios cooked, raw;

class Teleop {
private:
	ros::NodeHandle nh_;

	double l_min_, a_min_;
	double l_one_, a_one_;

	int l_step_, a_step_;
	int l_scale_, a_scale_;
	ros::Publisher twist_pub_;

public:
	Teleop(): nh_("teleop_key"),l_step_(0), a_step_(0)  {
		double l_max, a_max;
		nh_.getParam("max_line_speed", l_max);
		nh_.getParam("min_line_speed", l_min_);
		nh_.getParam("max_angular_speed", a_max);
		nh_.getParam("min_angular_speed", a_min_);
		nh_.getParam("scale_linear", l_scale_);
		nh_.getParam("scale_angular", a_scale_);
		
		ROS_DEBUG_STREAM( "l_max=" << l_max << " l_min_=" << l_min_ << " l_scale_=" << l_scale_ );
		ROS_DEBUG_STREAM( "a_max=" << a_max << " a_min_=" << a_min_ << " a_scale_=" << a_scale_ );
		l_one_ = (l_max - l_min_)/(double)l_scale_;
		a_one_ = (a_max - a_min_)/(double)a_scale_;

		twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	}

	void keyLoop() {
		char c;
		bool dirty=false;
		double linear, angular;

		// get the console in raw mode
		tcgetattr(kfd, &cooked);
		memcpy(&raw, &cooked, sizeof(struct termios));
		raw.c_lflag &=~ (ICANON | ECHO);
		// Setting a new line, then end of file
		raw.c_cc[VEOL] = 1;
		raw.c_cc[VEOF] = 2;
		tcsetattr(kfd, TCSANOW, &raw);

		puts("start Reading from keyboard...");

		for(;;) {
			linear=angular=0;
			// get the next event from the keyboard
			if(read(kfd, &c, 1) < 0) {
				perror("read():");
				exit(-1);
			}
			
			ROS_DEBUG("value: 0x%02X\n", c);

			switch(c) {
			case KEYCODE_A:
				ROS_DEBUG("LEFT");
				angular = a_min_+ a_one_*(double)a_step_;
				dirty = true;
				break;
			case KEYCODE_D:
				ROS_DEBUG("RIGHT");
				angular = -1.0*(a_min_+ a_one_*(double)a_step_);
				dirty = true;
				break;
			case KEYCODE_W:
				ROS_DEBUG_STREAM( "UP: l_min=" << l_min_ << " l_one=" << l_one_ << " l_step=" << l_step_ );
				linear = l_min_+ l_one_*(double)l_step_;
				dirty = true;
				break;
			case KEYCODE_X:
				ROS_DEBUG("DOWN");
				linear = -1.0*(l_min_+ l_one_*(double)l_step_);
				dirty = true;
				break;
			
			case KEYCODE_Q:
				ROS_DEBUG("Left up");
				linear = l_min_+ l_one_*(double)l_step_;
				angular = a_min_+ a_one_*(double)a_step_;
				dirty = true;
				break;
			case KEYCODE_E:
				ROS_DEBUG("right UP");
				linear = l_min_+ l_one_*(double)l_step_;
				angular = -1.0*(a_min_+ a_one_*(double)a_step_);
				dirty = true;
				break;	
			case KEYCODE_Z:
				ROS_DEBUG("left DOWN");
				linear = -1.0*(l_min_+ l_one_*(double)l_step_);
				angular = -1.0*(a_min_+ a_one_*(double)a_step_);
				dirty = true;
				break;	
			case KEYCODE_C:
				ROS_DEBUG("right DOWN");
				linear = -1.0*(l_min_+ l_one_*(double)l_step_);
				angular = a_min_+ a_one_*(double)a_step_;
				dirty = true;
				break;	
			
			case KEYCODE_S: case KEYCODE_SP: 
				ROS_DEBUG("STOP");
				linear = angular = 0;
				dirty = true;
				break;
			case KEYCODE_LV_D:
				ROS_DEBUG("Line speed down");
				l_step_--;
				if (l_step_ < 0) {
					l_step_ =0;
				}
				break;
			case KEYCODE_LV_U:
				ROS_DEBUG("Line speed up");
				l_step_++;
				if (l_step_ > l_scale_) {
					l_step_ = l_scale_;
				}
				break;
			case KEYCODE_AV_D:
				ROS_DEBUG("Angular speed down");
				a_step_--;
				if (a_step_ < 0) {
					a_step_ =0;
				}
				break;
			case KEYCODE_AV_U:
				ROS_DEBUG("Angular speed up");
				a_step_++;
				if (a_step_ > a_scale_) {
					a_step_ = a_scale_;
				}
				break;
			}

			geometry_msgs::Twist twist;
			twist.angular.z = angular;
			twist.linear.x = linear;
			if(dirty ==true) {
				twist_pub_.publish(twist);
				dirty=false;
			}
		}
	}
	
	void help() {
		std::cout << "Control Your robot by Keyboard!" << std::endl;
		std::cout << "---------------------------" << std::endl;
		std::cout << "Moving around:" << std::endl;
		std::cout << "q  w  e" << std::endl;
		std::cout << "a  s  d" << std::endl;
		std::cout << "z  x  c" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "_/+ : decrease/increase only linear speed by 10%" << std::endl;
		std::cout << "-/= : decrease/increase only angular speed by 10%" << std::endl;
		std::cout << "space key: force stop" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "CTRL-C to quit" << std::endl;
	}
};

void quit(int sig) {
	(void)sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_key");
	Teleop teleop;

	signal(SIGINT,quit);
	
	teleop.help();
	teleop.keyLoop();

	return(0);
}
