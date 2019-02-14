//************************************************************************************************************************************//
// From Navigator & Turner cpp files.
// Edited by: Darius Tan
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>

//********************** Info Reader CLASS **********************//
// WRT starting position X = 0, Y = 0, yaw = 0
// Checkerboard position = x: 10 y: -4
// Exit position = x: -0.6 y: -3
// Positive pose on map:	Yaw orientation on map:
//          X				//          0
//          |				//      +       -
//          |				//    +     .      -
//			|				//      +       -
// Y <------				//      3.14 -3.14

// 12 x 8 grid with start(S), goal(G), exit(E), walls (=, |)
//   |   =   =   =   =   =   =   =   |  12
//   |   *   *   *   *   *   *   *   |  11
//   |   *   *   *   *   *   G   *   |  10
//   |   *   *   *   *   *   *   *   |  9
//   |   *   =   =   =   =   =   =   |  8
//   |   *   *   *   *   *   *   *   |  7
//   |   *   *   *   *   *   *   *   |  6
//   |   *   *   *   *   *   *   *   |  5
//   |   =   =   =   =   =   =   *   |  4
//   |   *   *   *   *   *   *   *   |  3
//   |   *   *   *   *   *   *   *   |  2
//   |   *   *   *   *   *   *   *   |  1
//   |   =   S   =   =   E   =   =   |  0
//   8   7   6   5   4   3   2   1   0  

class InfoReader{
private:
	ros::Subscriber pos_sub;
	ros::Publisher pos_info;
	std_msgs::Float64MultiArray position;

public:
	InfoReader(ros::NodeHandle &nh){
		pos_sub = nh.subscribe("/odom",1, &InfoReader::callback, this);
		pos_info = nh.advertise< std_msgs::Float64MultiArray >("pos_info", 1);
	}

	void callback( const nav_msgs::OdometryConstPtr& poseMsg){
        // Take current X, Y positions, Z orientation
	
		double posX = poseMsg->pose.pose.position.x;
		double posY = 6.0 + poseMsg->pose.pose.position.y; // hardcoded to be offset by 6.

		// Quartenion processing
        // Use the turner.cpp quartenion processing methods than navigator.cpp (*2.19 hardcode)
        // Don't really need x, y, w orientations, only can rotate in z
		double quatX= poseMsg->pose.pose.orientation.x;
        double quatY= poseMsg->pose.pose.orientation.y;
        double quatZ= poseMsg->pose.pose.orientation.z;
        double quatW= poseMsg->pose.pose.orientation.w;
    	tf::Quaternion q(quatX, quatY, quatZ, quatW);
		tf::Matrix3x3 m(q);
		double yaw, pitch, roll;
        // Don't need roll and pitch, just take yaw.
    	m.getRPY(roll, pitch, yaw);

        // maybe change output instead of using Float64MultiArray
    	position.data.resize(3); //resize array to 3
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