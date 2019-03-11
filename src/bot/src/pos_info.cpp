//************************************************************************************************************************************//
// Node to return position estimates based on odometery values.
// From given Navigator & Turner C++ files.
// Edited by: Darius Tan
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

//********************** Info Reader CLASS **********************//
// Positive pose on map:	Yaw orientation on map:
//  Y						//          0
//  |						//      +       -
//  |						//    +     .      -
//	|						//      +       -
//  ------> X				//      3.14 -3.14

class InfoReader{
private:
	ros::Subscriber pos_sub;
	ros::Publisher pos_info;
	std_msgs::Float64MultiArray position;

public:
	InfoReader(ros::NodeHandle &nh){
		pos_sub = nh.subscribe("/robot_pose_ekf/odom_combined",1, &InfoReader::callback, this);
		pos_info = nh.advertise< std_msgs::Float64MultiArray >("pos_info", 1);
	}

	void callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg){
		// Changed positive pose on map:
		//  Y
		//  |
		//  |
		//	|
		//   -------> X
		double posX = -poseMsg->pose.pose.position.y;
		double posY = poseMsg->pose.pose.position.x;

		// Quartenion processing
		double quatX= poseMsg->pose.pose.orientation.x;
        double quatY= poseMsg->pose.pose.orientation.y;
        double quatZ= poseMsg->pose.pose.orientation.z;
        double quatW= poseMsg->pose.pose.orientation.w;
    	tf::Quaternion q(quatX, quatY, quatZ, quatW);
		tf::Matrix3x3 m(q);
		double yaw, pitch, roll;
    	m.getRPY(roll, pitch, yaw);

    	position.data.resize(3);

    	position.data[0] = posX;
    	position.data[1] = posY;
    	position.data[2] = yaw;

		// Publisher format -->
		// layout: 
		// 		dim: []
  		// 		data_offset: 0
		// data: [1.2506714127957979, 0.006914342738347928, -0.03538435380413812]

    	pos_info.publish(position);
        std::cout << "X Pos: " << posX 
                << " Y Pos: " << posY 
                << " Z Rot: " << yaw << std::endl;
	}
};

//********************** MAIN FUNCTION **********************//

int main(int argc, char** argv){
    ros::init(argc, argv, "pos_info");
    ros::NodeHandle nh;

    InfoReader ir(nh);

    ros::spin();
    return 0;
}