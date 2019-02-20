/* Turtlebot navigation for EE4308 Project Part 1 and Part 2 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "MazeSolver.h"
#include "semaphore.h"

//DIRECTION SIGNS and NUMBERING
//  8   1   5
//  4       2
//  7   3   6

//DIRECTION
static const int UP = 1;
static const int RIGHT = 2;
static const int DOWN = 3;
static const int LEFT = 4;
static const int NE = 5;
static const int SE = 6;
static const int SW = 7;
static const int NW = 8;
//USER INPUT
static const int GOAL_X = 4;
static const int GOAL_Y = 4;
static const double TARGET_DISTANCE = 1.0;
//CONSTANTS, THRESHOLDS, MINs, MAXes
static const double PI = 3.1415;
static const double ANGLE_THRESHOLD = 0.1;
static const double MINIMUM_DEPTH = 0.2;
static const double MAXIMUM_DEPTH = 0.5;
static const double YAW_THRESHOLD = 0.02;
static const double ALLOWED_DEVIATION = 0.005;
static const double GOAL_DEVIATION = 0.1;
static const double SEMAPHORE_SLOW_DOWN = 0.2;
static const double YAW_SPEED = 0.4;
static const double VELOCITY = 0.5;


int absRounder(double number){ //Round up and down to tolerate till 0.8
  double output = (number >= 0) ? number : number * -1;
  //if input is bigger than or equal to 0,
  //then output = number
  //else output = number *-1
  // abs(number)
  //std::cout<<"output : "<<output<<"___int(output) : "<<int(output)<<std::endl;
  return (output - 0.8 >= int(output)) ? int(output) + 1 : int(output);
  //if output - 0.8 is bigger than or equal to int(output)
  //then return int(output) + 1
  //else return int(output)
}

int validate(int chose){
  if(chose != 1 && chose != 2)
    return 1;
  return chose;
  //if chose is not 1 AND chose is not 2
  //then return 1
  //else return chose
  //---------------------------------------------------------------------
  //function is to validate whether user is inputting either 1 or 2
  //---------------------------------------------------------------------
}

class DataCollection{
private:
  MazeSolver maze; //calling an instance of MazeSolver called "maze"
  ros::Subscriber depth_sub;
  ros::Subscriber pose_sub;
  ros::Publisher teleop_pub;
  Semaphore semaphore;
  int algo_choice;
  double posx;
  double posy;
  double yaw;
  double prevDepth, depth;
  //For moving an yawing
  int move_count;
  double linear_velocity, init_x, init_y;
  double yaw_velocity;

public:
  //setup necessary variables and then validate choices
  //subscribed to "depth_info" and "position_info"
  //publish to teleop commands
  DataCollection(ros::NodeHandle &nh, int chose){
    maze = MazeSolver(9, 9, GOAL_X, GOAL_Y);
    semaphore.setUp();
    move_count = 0;
    linear_velocity = 0;
    algo_choice = validate(chose);
    depth_sub = nh.subscribe("depth_info", 1, &DataCollection::depthCallback, this);
    pose_sub = nh.subscribe("position_info", 1, &DataCollection::poseAndTeleopCallback, this);
    teleop_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
  }

  double finishRunningSemaphore(int direction){
    if(semaphore.isUp()){
      if(posx > absRounder(posx) + ALLOWED_DEVIATION) return -SEMAPHORE_SLOW_DOWN;
      if(posx < absRounder(posx) - ALLOWED_DEVIATION) return SEMAPHORE_SLOW_DOWN;
    }else if(semaphore.isRight()){
      if(-posy > absRounder(posy) + ALLOWED_DEVIATION) return -SEMAPHORE_SLOW_DOWN;
      if(-posy < absRounder(posy) - ALLOWED_DEVIATION) return SEMAPHORE_SLOW_DOWN;
    }else if(semaphore.isDown()){
      if(posx > absRounder(posx) + ALLOWED_DEVIATION) return SEMAPHORE_SLOW_DOWN;
      if(posx < absRounder(posx) - ALLOWED_DEVIATION) return -SEMAPHORE_SLOW_DOWN;
    }else if(semaphore.isLeft()){
      if(-posy > absRounder(posy) + ALLOWED_DEVIATION) return SEMAPHORE_SLOW_DOWN;
      if(-posy < absRounder(posy) - ALLOWED_DEVIATION) return -SEMAPHORE_SLOW_DOWN;
    }
    //function, input direction
    //SEMAPHORE_SLOW_DOWN = 0.2

    if(direction!=0) {
      semaphore.setSemaphore(direction);
      if(absRounder(posx)!=absRounder(init_x) || absRounder(posy)!=absRounder(init_y))
        maze.setLastPosition(absRounder(init_x), absRounder(init_y));
    }
    //if direction is not 0, then setSemaphore will setup the directional booleans accordingly
    //and if the current posx and poxy is not the init x and y, then the coordinates will be set

    move_count = 0;
    return 0;
  }

  bool faceDirection(int direction){
    switch(direction){
      case UP:
        return (yaw >= -ANGLE_THRESHOLD && yaw <= ANGLE_THRESHOLD);
      case RIGHT:
        return (yaw >= -PI/2 - ANGLE_THRESHOLD && yaw <= -PI/2 + ANGLE_THRESHOLD);
      case DOWN:
        return ((yaw >= PI - ANGLE_THRESHOLD && yaw <= PI) || (yaw <= -PI + ANGLE_THRESHOLD && yaw >= -PI));
      case LEFT:
        return (yaw >= PI/2 - ANGLE_THRESHOLD && yaw <= PI/2 + ANGLE_THRESHOLD);
      default:
        return false;
    }
  }

  double generateYawUnitAngle(int direction){
    switch(direction){
      case UP:
        if (yaw <= -ANGLE_THRESHOLD){
          return YAW_SPEED;
        }else{
          return -YAW_SPEED;
        }
      case RIGHT:
        if (yaw >= -PI/2 + ANGLE_THRESHOLD && yaw <= PI/2){
          return -YAW_SPEED;
        }else{
          return YAW_SPEED;
        }
      case DOWN:
        if (yaw >= -PI + ANGLE_THRESHOLD && yaw <= 0){
          return -YAW_SPEED;
        }else{
          return YAW_SPEED;
        }
      case LEFT:
        if (yaw <= PI/2 - ANGLE_THRESHOLD && yaw >= -PI/2){
          return YAW_SPEED;
        }else{
          return -YAW_SPEED;
        }
      default:
        return 0;
    }
  }

  double yawCorrection(double angle){
    if(angle == PI){
      if(yaw <= -PI + YAW_THRESHOLD && yaw >= -PI) return YAW_SPEED;
      if(yaw >= PI - YAW_THRESHOLD && yaw <= PI) return -YAW_SPEED;
    }else{
      if(yaw <= angle - YAW_THRESHOLD) return YAW_SPEED;
      if(yaw >= angle + YAW_THRESHOLD) return -YAW_SPEED;
    }
    return 0;
  }

  double straightenTrajectory(int direction){
    switch(direction){
      case UP:
        return yawCorrection(0.0);
      case RIGHT:
        return yawCorrection(-PI/2);
      case DOWN:
        return yawCorrection(PI);
      case LEFT:
        return yawCorrection(PI/2);
      default:
        return 0;
    }
  }

  double goStraight(int direction){
    switch(direction){
      case UP:
        if(posx - init_x < TARGET_DISTANCE) return VELOCITY;
        break;
      case DOWN:
        if(init_x - posx < TARGET_DISTANCE) return VELOCITY;
        break;
      case RIGHT:
        if(init_y - posy < TARGET_DISTANCE) return VELOCITY;
        break;
      case LEFT:
        if(posy - init_y < TARGET_DISTANCE) return VELOCITY;
        break;
      default:
        break;
    }
    if(absRounder(posx)!=absRounder(init_x) || absRounder(posy)!=absRounder(init_y))
      maze.setLastPosition(absRounder(init_x), absRounder(init_y));
    move_count = 0;
    return 0;
  }

  //this function handles all data acquired from depth_info
  void depthCallback(const std_msgs::Float64::ConstPtr& depth_data){
    if (isnan(depth_data->data)){
      depth = prevDepth;
    } else{
      //With 80% confidence that if wall exists, it will be a low number and the other way round.
      prevDepth = depth;
      depth = depth_data->data;
    }
  }



  //########################################################################################################
  //FUNCTION THAT RECEIVES THE POSITION_INFO FROM SUBSRIPTION
  //########################################################################################################
  void poseAndTeleopCallback(const std_msgs::Float64MultiArray::ConstPtr& pose_data){
    posx = pose_data->data[0];
    posy = pose_data->data[1];
    yaw = pose_data->data[2];
    //Print for confirmation
    //----------------------------------------------------------------------------------------------------------------------
    //std::cout << "Pos x -> " << posx << " Pos y -> " << posy << " Yaw -> " << yaw << " Depth -> " << depth << std::endl;
    //----------------------------------------------------------------------------------------------------------------------
    /* Where the path planning happens
     * Take note that turtlebot can only go straight and yaw (non-holonomic constraint: cannot slide or fly).
     * The only teleop command it takes is linear.x and angular.z
     */
    //Moving and turning preprocessing, maze computing
    geometry_msgs::Twist velocity;
    if(move_count == 0){
      init_x = posx;
      init_y = posy;
      move_count = 1;
    }
    //^this initialization will not happen unless move_count returns back to 0

    maze.resetGridValue(); //reset all f and g grid values to 555

    //terminating condition
    if((posx >= GOAL_X - GOAL_DEVIATION && posx<= GOAL_X + GOAL_DEVIATION) &&
      (posy >= -GOAL_Y - GOAL_DEVIATION && posy <= -GOAL_Y + GOAL_DEVIATION)){
        //reminder, goal_x and goal_y is 4, goal_deviation is 0.1
        //simply means if the posx and posy is within range of goal x and goal y
      std::cout << "reachGoal" <<std::endl;
      linear_velocity = 0;
      yaw_velocity = 0;
      //stop moving and stop turning
    }else{
      int nextMove = maze.navigation(absRounder(posx), absRounder(posy), algo_choice);
      //maze.navigation function returns algorithm depending on algo_choice
      //algo_choice is also validated from the variable 'chose' above.
      //absRounder posx posy is inputted into algorithm's function as chosen by user
      std::cout<<"Next Move: "<< nextMove << std::endl;

      if(!semaphore.checkIfItIs(nextMove)){ //Make sure semaphore is given to next move
        //if check the current direction boolean using nextMove to represent direction
        linear_velocity = finishRunningSemaphore(nextMove);
      }else if(!faceDirection(nextMove)){ //Yaw towards the direction
        linear_velocity = 0;
        yaw_velocity = generateYawUnitAngle(nextMove);
      }else if(depth >= MINIMUM_DEPTH && depth <= MAXIMUM_DEPTH){ //Build a wall if found
        linear_velocity = 0;
        maze.buildWall(absRounder(posx), absRounder(posy), nextMove, true);
      }else{ //Move straightly
        yaw_velocity = straightenTrajectory(nextMove);
        linear_velocity = goStraight(nextMove);
      }
    }
    velocity.linear.x = linear_velocity;
    velocity.angular.z = yaw_velocity;
    teleop_pub.publish(velocity);
  }
};

int main(int argc, char **argv){
  std::cout<<"Choose the algorithm - \n1 - Flood-fill\n2 - A-Star"<<std::endl;
  int choice;
  std::cin >> choice;
  //take in user choice


  ros::init(argc, argv, "turtle_brain");
  ros::NodeHandle nh;

  DataCollection currData(nh, choice);
  //main command ^

  ros::spin();
  return 0;
}
