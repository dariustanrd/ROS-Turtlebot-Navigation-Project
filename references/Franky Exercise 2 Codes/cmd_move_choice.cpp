#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <queue>
#include <armadillo>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char choose();

/** declare the coordinates of interest **/
double xCafe = -6.2;
double yCafe = -2.73;
double xOffice1 = 0.0629 ;
double yOffice1 = 4.26;
double xOffice2 = 2.72 ;
double yOffice2 = 2.65;

double dist_mid,dist_left,dist_right,distMsg;
double target_dist=20000,dist_limit;

bool goalReached = false;

void distCallback(const std_msgs::Float32MultiArray& distMsg)
{
	dist_mid = distMsg.data[1];
	dist_left = distMsg.data[0];
	dist_right = distMsg.data[2];
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::Subscriber sub_scan = n.subscribe("/distance_logger/distances", 1, distCallback);
	ros::Publisher pub_action_choice = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

	ros::spinOnce();
	
	geometry_msgs::Twist cmd_move;
	//cmd_move.linear.x = ;
	//cmd_move.angular.z = ;
	//pub_action_choice.publish(cmd_move);	

	char choice = 'q';
	do{
		choice =choose();
		dist_limit = target_dist;
		if (choice == 'w'){
			while (dist_limit > 0)		
			{	
				if(abs(dist_right - dist_left) < 0.1)
				{
					cmd_move.linear.x = 0.1;
					cmd_move.angular.z = 0;
					pub_action_choice.publish(cmd_move);
				}
				else if(dist_right - dist_left >= 0.1)
				{
					cmd_move.linear.x = 0.1;
					cmd_move.angular.z = -0.1;
					pub_action_choice.publish(cmd_move);
				}
				else if(dist_right - dist_left <= -0.1)
				{
					cmd_move.linear.x = 0.1;
					cmd_move.angular.z = 0.1;
					pub_action_choice.publish(cmd_move);
				}
				dist_limit = dist_limit-0.1;
				std::cout<<target_dist<<"____"<<dist_limit<<"____"<<dist_right<<std::endl;			
			}		
		}else if (choice == 'a'){
			cmd_move.linear.x = 0;
			cmd_move.angular.z = 2.64;
			pub_action_choice.publish(cmd_move);
		}else if (choice == 'd'){
			cmd_move.linear.x = 0;
			cmd_move.angular.z = -2.64;
			pub_action_choice.publish(cmd_move);
		}else if (choice == 's'){
			cmd_move.linear.x = -1;
			cmd_move.angular.z = 0;
			pub_action_choice.publish(cmd_move);
		}if (choice!='q'){
			if (goalReached){
				ROS_INFO("Congratulations!");
				ros::spinOnce();
				ros::spinOnce();

			}else{
				ROS_INFO("Hard Luck!");
			}
		}
	}while(choice !='q');

	return 0;
}


char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|PRESS A KEY:"<<std::endl;
	std::cout<<"|'w': Forward "<<std::endl;
	std::cout<<"|'a': Turn Left "<<std::endl;
	std::cout<<"|'s': Reverse "<<std::endl;
	std::cout<<"|'d': Turn Right "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|WHERE TO GO?";
	std::cin>>choice;

	return choice;


}

