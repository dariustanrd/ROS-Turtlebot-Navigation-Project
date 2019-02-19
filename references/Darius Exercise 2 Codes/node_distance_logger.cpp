/* This node returns an array with distances from the scan topic:
The leftmost measured distance, middle distance and rightmost distance in
the new topic distance_logger/distances which is a message of type
Float32MultiArray. The distances is stored in the "data" sub-message
within the topic, as [left, middle, right]
 If the message from /scan returns "nan", this node will either set the distance
 to 0.01 (wall) or 10 (distance too far for the sensor to see).
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <armadillo>
#include <std_msgs/Float32MultiArray.h>

using namespace arma;

class DistanceLogger{
private:
	ros::Subscriber sub_scan;
	ros::Publisher pub_distance_logger;
	std::vector<float> scan_vector;
	bool initialized;
	double max_dist, min_dist;
	float scan_left, scan_middle, scan_right;
	float scan_left_prev, scan_middle_prev, scan_right_prev;
public:
	DistanceLogger(ros::NodeHandle &nh);
	void callback_distance_logger(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
};


DistanceLogger::DistanceLogger(ros::NodeHandle &nh) {
	//sub_scan = nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&BotController::processLaserScan,this);
	sub_scan = nh.subscribe("/scan", 1, &DistanceLogger::callback_distance_logger, this);
	pub_distance_logger = nh.advertise<std_msgs::Float32MultiArray>("/distance_logger/distances",1);
	max_dist = 10;
	min_dist = 0.01;
	scan_left_prev = 3;
	scan_middle_prev = 3;
	scan_right_prev = 3;
	initialized = false;
}


void DistanceLogger::callback_distance_logger(const sensor_msgs::LaserScan::ConstPtr& scanMsg){

	scan_vector = scanMsg->ranges;
	int scan_vector_middle_index = (int)round(scan_vector.size() / 2.0);
	scan_right = scan_vector.front();
	scan_middle = scan_vector[scan_vector_middle_index];
	scan_left = scan_vector.back();

	std::cout << scan_left << "__;__" << scan_middle << "__;__" << scan_right<< std::endl;

	if (isnanf(scan_left)){
		//std::cout << "nan value detected left" << std::endl;
		if(!initialized){
			scan_left = max_dist;
		}
		else if(scan_left_prev > 2){
			scan_left = max_dist;
		}
		else if(scan_left_prev <= 2){
			scan_left = min_dist;
		}
	}

	if (isnanf(scan_middle)){
		//std::cout << "nan value detected middle" << std::endl;
		if(!initialized){
			scan_middle = max_dist;
		}
		else if(scan_middle_prev > 2){
			scan_middle = max_dist;
		}
		else if(scan_middle_prev <= 2){
			scan_middle = min_dist;
		}
	}

	if (isnanf(scan_right)){
		//std::cout << "nan value detected right" << std::endl;
		if(!initialized){
			utput. Clever!

With a heuristic, we need to make sure that we can actually calculate it. It’s also very important that the heuristic is always an underestimation of the total path, as an overestimation will lead to A* searching for through nodes that may not be the ‘best’ in terms of f value.

scan_right = max_dist;
		}
		else if(scan_right_prev > 2){
			scan_right = max_dist;
		}
		else if(scan_left_prev <= 2){
			scan_right = min_dist;
		}
	}

	if(!initialized){
		initialized = true;
	}

	scan_left_prev = scan_left;
	scan_middle_prev = scan_middle;
	scan_right_prev = scan_right;

	//std::cout << "------------------------------" << std::endl;
	std_msgs::Float32MultiArray cmd_distances;
	cmd_distances.data.clear();
	cmd_distances.data.push_back(scan_left);
	cmd_distances.data.push_back(scan_middle);
	cmd_distances.data.push_back(scan_right);

	pub_distance_logger.publish(cmd_distances);
	std::cout
}

int main(int argc, char** argv){
	ros::init(argc, argv, "node_distance_logger");
	ros::NodeHandle nh;

	DistanceLogger dl(nh);
	ros::spin();
	return 0;
}
