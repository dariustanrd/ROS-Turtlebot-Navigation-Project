//************************************************************************************************************************************//
// Node to return depth information from Laserscan for Left, Middle, Right
// From online sources
// Edited by: Darius Tan & Franky Laurentis
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <deque> //testing

#define HISTBUF 10

static const float MAX_DEPTH = 5.0;
static const float MIN_DEPTH = 0.4;

class depthScanner {
	private:
		ros::Subscriber scan_sub;
		ros::Publisher scan_info;
		std::vector<float> scan_vect;

		std_msgs::Float64MultiArray scan_msg;
		float scan_left, scan_mid, scan_right;
		float send_left, send_mid, send_right;

		float minRan, maxRan, minAng, maxAng;

		bool startup;

		std::deque<float> scanHistLeft;
		std::deque<float> scanHistMid;
		std::deque<float> scanHistRight;

	public:
    depthScanner(ros::NodeHandle &nh){
		scan_sub = nh.subscribe("/scan", 1, &depthScanner::scanCallback, this);
		scan_info = nh.advertise< std_msgs::Float64MultiArray >("scan_info", 1);
		startup = true;
		for (int i = 0; i < HISTBUF; ++i) {
			scanHistLeft.push_back(0);
			scanHistMid.push_back(0);
			scanHistRight.push_back(0);
		}
	}
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
		// float32 range_min        # minimum range value [m]
		// float32 range_max        # maximum range value [m]
		// float32 angle_min        # start angle of the scan [rad]
		// float32 angle_max        # end angle of the scan [rad]
		scan_vect = scan->ranges;
		minRan = scan->range_min;
		maxRan = scan->range_max;
		minAng = scan->angle_min;
		maxAng = scan->angle_max;

		// std::cout << "minRan: " << minRan << " maxRan: " << maxRan << " minAng: " << minAng << " maxAng: " << maxAng << std::endl;

		int midIdx = (int)round(scan_vect.size() / 2.0);
		scan_right = scan_vect.front();
		scan_mid = scan_vect[midIdx];
		scan_left = scan_vect.back();

		if (startup) {
			if (std::isnan(scan_left))
				send_left = MAX_DEPTH;
			if (std::isnan(scan_mid))
				send_mid = MAX_DEPTH;
			if (std::isnan(scan_right))
				send_right = MAX_DEPTH;
			scanHistLeft.push_back(send_left);
			scanHistMid.push_back(send_mid);
			scanHistRight.push_back(send_right);
		}
		else {
			if (std::isnan(scan_left)) {
				if (*scanHistLeft.begin() < *scanHistLeft.rbegin()) //if was previously increasing
					send_left = MAX_DEPTH;
				else if (*scanHistLeft.begin() == *scanHistLeft.rbegin())
					send_left = send_left;
				else
					send_left = MIN_DEPTH;
			} 
			else
				send_left = scan_left;
			if (std::isnan(scan_mid)) {
				if (*scanHistMid.begin() < *scanHistMid.rbegin())
					send_mid = MAX_DEPTH;
				else if (*scanHistMid.begin() == *scanHistMid.rbegin())
					send_mid = send_mid;
				else
					send_mid = MIN_DEPTH;
			} 
			else
				send_mid = scan_mid;
			if (std::isnan(scan_right)) {
				if (*scanHistRight.begin() < *scanHistRight.rbegin())
					send_right = MAX_DEPTH;
				else if (*scanHistRight.begin() == *scanHistRight.rbegin())
					send_right = send_right;
				else
					send_right = MIN_DEPTH;
			} 
			else
				send_right = scan_right;
			scanHistLeft.pop_front();
			scanHistMid.pop_front();
			scanHistRight.pop_front();

			scanHistLeft.push_back(send_left);
			scanHistMid.push_back(send_mid);
			scanHistRight.push_back(send_right);
		}
		startup = false;
		std::cout << "Send Left: " << send_left << " Send Mid: " << send_mid << " Send Right: " << send_right << std::endl;

		scan_msg.data.resize(3);
		scan_msg.data[0] = send_left;
		scan_msg.data[1] = send_mid;
		scan_msg.data[2] = send_right;
		scan_info.publish(scan_msg);
	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "scan_info");
	ros::NodeHandle nh;

	depthScanner ds(nh);
	ros::spin();
	return 0;
}