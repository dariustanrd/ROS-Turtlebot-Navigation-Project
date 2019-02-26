#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>

static const float MAX_DEPTH = 5.0;
static const float MIN_DEPTH = 0.4;

class depthScanner {
	private:
		ros::Subscriber scan_sub;
		ros::Publisher scan_info;
		std::vector<float> scan_vect;
    
		std_msgs::Float64MultiArray scan_msg;
		float scan_left, scan_mid, scan_right, leftPrev, midPrev, rightPrev;
		float send_left, send_mid, send_right;

	public:
    depthScanner(ros::NodeHandle &nh){
			scan_sub = nh.subscribe("/scan", 1, &depthScanner::callback, this);
			scan_info = nh.advertise< std_msgs::Float64MultiArray >("scan_info", 1);
		}
    void callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
			scan_vect = scan->ranges;
			int midIdx = (int)round(scan_vect.size() / 2.0);
			scan_right = scan_vect.front();
			scan_mid = scan_vect[midIdx];
			scan_left = scan_vect.back();


			// TODO: if scan value more than 3, set as max. if scan value less than 0.5 set as min
			// publish the fixed value

			std::cout << "Scan Left: " << scan_left << " Scan Mid: " << scan_mid << " Scan Right: " << scan_right << std::endl;

			if(scan_left > 4.5) send_left = MAX_DEPTH;
			else if (scan_left < 0.45) send_left = MIN_DEPTH;
			else send_left = scan_left;

			if(scan_left > 4.5) send_left = MAX_DEPTH;
			else if (scan_left < 0.45) send_left = MIN_DEPTH;
			else send_left = scan_left;

			if(scan_left > 4.5) send_left = MAX_DEPTH;
			else if (scan_left < 0.45) send_left = MIN_DEPTH;
			else send_left = scan_left;

			// if(std::isnan(scan_left)) {
			// 	if (leftPrev >= 2.0) scan_left = MAX_DEPTH; // prev value large, means going to max
			// 	else scan_left = MIN_DEPTH; // prev value small, means going to min
			// }
			// if(std::isnan(scan_mid)) {
			// 	if (midPrev >= 2.0) scan_mid = MAX_DEPTH; // prev value large, means going to max
			// 	else scan_mid = MIN_DEPTH; // prev value small, means going to min
			// }
			// if(std::isnan(scan_right)) {
			// 	if (rightPrev >= 2.0) scan_right = MAX_DEPTH; // prev value large, means going to max
			// 	else scan_right = MIN_DEPTH; // prev value small, means going to min
			// }
			
			// Publisher format -->
			// layout:
			// 		dim: []
			// 		data_offset: 0
			// data: [1.2506714127957979, 0.006914342738347928, -0.03538435380413812]
			std::cout << "Send Left: " << send_left << " send Mid: " << send_mid << " send Right: " << send_right << std::endl;

			scan_msg.data.resize(3); //resize array to 3
    	// scan_msg.data[0] = scan_left;
    	// scan_msg.data[1] = scan_mid;
    	// scan_msg.data[2] = scan_right;
			scan_msg.data[0] = send_left;
    	scan_msg.data[1] = send_mid;
    	scan_msg.data[2] = send_right;
			scan_info.publish(scan_msg);
			std::cout << std::endl;
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "scan_info");
	ros::NodeHandle nh;

	depthScanner ds(nh);
	ros::spin();
	return 0;
}