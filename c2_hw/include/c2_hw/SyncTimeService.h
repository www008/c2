#include <stdexcept>
#include <time.h>

// ROS
#include <ros/ros.h>

#include "c2_hw/SyncTime.h"

namespace c2_hw {

class SyncTimeService {
protected:
	ros::ServiceServer service_;
	
public:
	bool registerService(ros::NodeHandle& nh) {
		service_ = nh.advertiseService("sync_time", &SyncTimeService::callback, this);
	}
 
	bool callback(SyncTime::Request  &req, SyncTime::Response &resp) {
		struct timespec ts;
		if (!clock_gettime(CLOCK_REALTIME, &ts)) {
			resp.server_time_req = ts.tv_sec;
			resp.server_time_resp = ts.tv_sec;
		}
		return true;
	}
};

}
