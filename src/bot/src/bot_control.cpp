//************************************************************************************************************************************//
// Main controller file. To include search algorithms for pathfinding.
// Edited by: Darius Tan & Franky Laurentis
//************************************************************************************************************************************//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

#include "algo.h"

// WRT starting position X = 0, Y = 0, yaw = 0
// Goal position = x: 4 y: 4
// Positive pose on map:	Yaw orientation on map:
//  Y						//          0
//  |						//      +       -
//  |						//    +     .      -
//	|						//      +       -
//  ------> X				//      3.14 -3.14

static const double MIN_DEPTH = 0.40;
static const double MAX_DEPTH = 5.0;
static const double PI = 3.1415;
static const double ANG_ERR = 0.1;
// static const double DEPTH_LIM = 0.7;
static const double DEPTH_LIM = 0.5;

static const double POSE_TOLERANCE = 0.1;

static const double MOVE_DIST = 1.0;
static const double LIN_VEL = 0.5;
static const double YAW_VEL = 0.4;
static const double YAW_VEL_CORRECTION = 0.1;

int myRound (double num) {
    int out = 0;
    if (num >= 0) {
        if (num - int(num) >= 0.7) {
            out = int(num);
            out++;
            return out;
        } else return int(num);
    } else if (num < 0) {
        double tmp = -num;
        if (tmp - int(tmp) >= 0.7) {
            out = int(num);
            out--;
            return out;
        } else return int(num);
    }
}

class BotController
{
  private:
    ros::Subscriber depth_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber pose_sub;
    ros::Publisher cmd_pub;
    double posX, posY;
    double yaw;
    double depth;
    double laserL, laserR;

    bool preemptedWall;
    int numMoves;

    geometry_msgs::Twist cmd;
    double linear_cmd, yaw_cmd;
    double initX, initY;
    bool movedFlag;
    ros::Time begin;
    pathfinderAlgo algo;
    coord curCoord, nextCoord;

    bool reachedGoalOnce;

  public:
    BotController(ros::NodeHandle &nh) {
        movedFlag = true;
        linear_cmd = 0;
        yaw_cmd = 0;
        depth = MAX_DEPTH;
        laserL = MAX_DEPTH;
        laserR = MAX_DEPTH;
        preemptedWall = false;
        reachedGoalOnce = false;
        numMoves = 0;

        curCoord = startCoord;
        algo = pathfinderAlgo();
        if (ALGO == 1) {
            nextCoord = algo.aStar(startCoord);
        } else if (ALGO == 0) {
           // For FF
           nextCoord = algo.floodFill(startCoord); 
        }

        depth_sub = nh.subscribe("depth_info", 1, &BotController::depthCallback, this);
        laser_sub = nh.subscribe("scan_info", 1, &BotController::laserCallback, this);
        pose_sub = nh.subscribe("pos_info", 1, &BotController::poseCallback, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
        begin = ros::Time::now();
    }

    void laserCallback(const std_msgs::Float64MultiArray::ConstPtr& laser_data) {
        laserL = laser_data -> data[0];
        laserR = laser_data -> data[2];
        // std::cout << "laserL: " << laserL << "laserR: " << laserR << std::endl;
    }

    void depthCallback(const std_msgs::Float64::ConstPtr& depth_data) {
        depth = depth_data -> data;
        if (std::isnan(depth)) {
            depth = MIN_DEPTH;
        }
        // std::cout << "depth: " << depth << std::endl;
    }

    bool checkObs() {
        // return true if obs detected
        if (depth <= DEPTH_LIM) {
            // std::cout << "Block depth: " << depth << std::endl;
            return true;
        } else return false;
    }

    void moveStr(int direction) {
        switch (direction) {
        case STOP:
            linear_cmd = 0;
            break;
        case UP:
            if (posY - initY < MOVE_DIST) {
                linear_cmd = LIN_VEL;
                // std::cout << "Straight UP" << std::endl;
            }
            if (posX >= curCoord.first){
                if (yaw <= 0){
                    yaw_cmd = YAW_VEL_CORRECTION;
                }
            }
            else if (posX <= curCoord.first){
                if (yaw >= 0){
                    yaw_cmd = -YAW_VEL_CORRECTION;
                }
            }
            break;
        case DOWN:
            if (initY - posY < MOVE_DIST) {
                linear_cmd = LIN_VEL;
                // std::cout << "Straight DOWN" << std::endl;
            }
            if (posX >= curCoord.first){
                if (yaw <0 && yaw > -PI){
                    yaw_cmd = -YAW_VEL_CORRECTION;
                }
            }
            else if (posX <= curCoord.first){
                if (yaw > 0 && yaw <= PI){
                    yaw_cmd = YAW_VEL_CORRECTION;
                }
            }
            break;
        case LEFT:
            if (initX - posX < MOVE_DIST) {
                linear_cmd = LIN_VEL;
                // std::cout << "Straight LEFT" << std::endl;
            }
            if (posY >= curCoord.second) {
                if (yaw <= PI/2){
                    yaw_cmd = YAW_VEL_CORRECTION;
                }
            }
            else if (posY < curCoord.second) {
                if (yaw >= PI/2){
                    yaw_cmd = -YAW_VEL_CORRECTION;
                }
            }
            break;
        case RIGHT:
            if (posX - initX < MOVE_DIST) {
                linear_cmd = LIN_VEL;
                // std::cout << "Straight RIGHT" << std::endl;
            }
            if (posY >= curCoord.second) {
                if (yaw >= -PI/2){
                    yaw_cmd = -YAW_VEL_CORRECTION;
                }
            }
            else if (posY < curCoord.second) {
                if (yaw <= -PI/2){
                    yaw_cmd = YAW_VEL_CORRECTION;
                }
            }
            break;
        default:
            break;
        }
        // For FF
        if (ALGO == 0) {
            if(myRound(posX)!=myRound(initX) || myRound(posY)!=myRound(initY)) 
                algo.setLastPosition(myRound(initX), myRound(initY));
        }
        cmd.angular.z = yaw_cmd;
        cmd.linear.x = linear_cmd;
        cmd_pub.publish(cmd);
    }

    bool checkFace (int direction) {
        // std::cout << "Checking Face" << std::endl;
        switch (direction) {
            case UP:
                // if current yaw near 0
                return (0 - ANG_ERR <= yaw && yaw <= 0 + ANG_ERR);
                break;
            case DOWN:
                // if current yaw near PI | -PI
                return ((PI - ANG_ERR <= yaw && yaw <= PI + ANG_ERR) || (-PI - ANG_ERR <= yaw && yaw <= -PI + ANG_ERR));
                break;
            case LEFT:
                // if current yaw near PI/2
                return (PI/2 - ANG_ERR <= yaw && yaw <= PI/2 + ANG_ERR);
                break;
            case RIGHT:
                // if current yaw near -PI/2
                return (-PI/2 - ANG_ERR <= yaw && yaw <= -PI/2 + ANG_ERR);
                break;
            default:
                return 0;
        }
    }

    void moveFace (int direction) {
        // Positive YAW_VEL -> ACW. Negative YAW_VEL -> CW.
        switch (direction) {
            case STOP:
                // set yaw_cmd to 0;
                yaw_cmd = 0;
                break;
            case UP:
                // if current yaw near 0, stop. else if yaw negative, rotate ACW, if yaw positive, rotate CW
                if (checkFace(UP)) {
                    // std::cout << "FACE UP OK" << std::endl;
                    yaw_cmd = 0;
                } else {
                    // std::cout << "MOVE FACE UP" << std::endl;
                    if (yaw > 0) yaw_cmd = -YAW_VEL; //CW
                    if (yaw < 0) yaw_cmd = YAW_VEL; //ACW
                }
                break;
            case DOWN:
                // if current yaw near PI | -PI, stop. else if yaw negative, rotate CW, if yaw positive, rotate ACW
                if (checkFace(DOWN)) {
                    // std::cout << "FACE DOWN OK" << std::endl;
                    yaw_cmd = 0;
                } else {
                    // std::cout << "MOVE FACE DOWN" << std::endl;
                    if (yaw < 0) yaw_cmd = -YAW_VEL; //CW
                    if (yaw > 0) yaw_cmd = YAW_VEL; //ACW
                }
                break;
            case LEFT:
                // if current yaw near PI/2, stop. else if -PI/2 < yaw < PI /2, rotate ACW, if -PI < yaw < -PI/2 || PI/2 < yaw < PI, rotate CW
                if (checkFace(LEFT)) {
                    // std::cout << "FACE LEFT OK" << std::endl;
                    yaw_cmd = 0;
                } else {
                    // std::cout << "MOVE FACE LEFT" << std::endl;
                    if ((-PI < yaw && yaw < -PI/2) || (PI/2 < yaw && yaw < PI)) yaw_cmd = -YAW_VEL; //CW
                    if (-PI/2 < yaw && yaw < PI /2) yaw_cmd = YAW_VEL; //ACW
                }
                break;
            case RIGHT:
                // if current yaw near -PI/2, stop. else if -PI/2 < yaw < PI /2, rotate CW, if -PI < yaw < -PI/2 || PI/2 < yaw < PI, rotate ACW
                if (checkFace(RIGHT)) {
                    // std::cout << "FACE RIGHT OK" << std::endl;
                    yaw_cmd = 0;
                } else {
                    // std::cout << "MOVE FACE RIGHT" << std::endl;
                    if (-PI/2 < yaw && yaw < PI /2) yaw_cmd = -YAW_VEL; //CW
                    if ((-PI < yaw && yaw < -PI/2) || (PI/2 < yaw && yaw < PI)) yaw_cmd = YAW_VEL; //ACW
                }
                break;
            default:
                break;
        }
        cmd.angular.z = yaw_cmd;
        cmd_pub.publish(cmd);
    }

    coord getCoord(double posX, double posY) {
        coord curCoord (myRound(posX), myRound(posY));
        return curCoord;
    }

    int checkCoordFace (coord curCoord, coord nextCoord) {
        // Assuming that pathfinder will only give next coord in 4 directions wrt current.
        if (nextCoord.second > curCoord.second) {
            return UP;
        }
        else if (nextCoord.second < curCoord.second) {
            return DOWN;
        }
        else if (nextCoord.first > curCoord.first) {
            return RIGHT;
        }
        else if (nextCoord.first < curCoord.first) {
            return LEFT;
        }
    }

    bool checkBotReached(double x, double y, coord destCoord) {
        if ((x <= destCoord.first + POSE_TOLERANCE) && (x >= destCoord.first - POSE_TOLERANCE)) { // if cur x position within +- tolerance of destination x coord
            if ((y <= destCoord.second + POSE_TOLERANCE) && (y >= destCoord.second - POSE_TOLERANCE)) { // if cur x position within +- tolerance of destination y coord
                return true;
            }
        }
        return false;
    }

    bool movedDistance(int direction, double dist) {
        switch (direction) {
            case UP:
                if (posY - curCoord.second > dist)
                    return true;
                break;
            case DOWN:
                if (curCoord.second - posY > dist)
                    return true;
                break;
            case LEFT:
                if (curCoord.first - posX > dist)
                    return true;
                break;
            case RIGHT:
                if (posX - curCoord.first > dist)
                    return true;
                break;
        }
        return false;
    }

    void preemptWall(int direction) {
        // then when it reaches the next coord and runs aStar again, it will realise there is a wall somewhere without turning towards it.
        
        // std::cout << "Preempting wall along direction: " << direction << std::endl;
        switch (direction) {
            case UP:
            // std::cout << "In case UP" << std::endl;
                if (laserL <= 1) {
                    algo.updateWall(nextCoord, LEFT, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, LEFT, false);
                }
                if (laserR <= 1) {
                    algo.updateWall(nextCoord, RIGHT, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, RIGHT, false);
                }
                break;
            case DOWN:
            // std::cout << "In case DOWN" << std::endl;
                if (laserL <= 1) {
                    algo.updateWall(nextCoord, RIGHT, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, RIGHT, false);
                }
                if (laserR <= 1) {
                    algo.updateWall(nextCoord, LEFT, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, LEFT, false);
                }
                break;
            case LEFT:
            // std::cout << "In case LEFT" << std::endl;
                if (laserL <= 1) {
                    algo.updateWall(nextCoord, DOWN, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, DOWN, false);
                }
                if (laserR <= 1) {
                    algo.updateWall(nextCoord, UP, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, UP, false);
                }
                break;
            case RIGHT:
            // std::cout << "In case RIGHT" << std::endl;
                if (laserL <= 1) {
                    algo.updateWall(nextCoord, UP, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, UP, false);
                }
                if (laserR <= 1) {
                    algo.updateWall(nextCoord, DOWN, true);
                    // std::cout << "Updated wall for coord: " << nextCoord.first << "," << nextCoord.second << std::endl;
                } else { 
                    algo.updateWall(nextCoord, DOWN, false);
                }
                break;
        }
    }
    void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& pose_data) {
        posX = pose_data -> data[0];
        posY = pose_data -> data[1];
        yaw = pose_data -> data[2];
        // std::cout << "Pos X: " << posX << "Pos Y: " << posY  << "Yaw: " << yaw << std::endl;
        if (movedFlag) {
            initX = posX;
            initY = posY;
            movedFlag = false;
        }
        int nextStep;
        algo.resetGrid();

        if (checkBotReached(posX,posY,goalCoord)) {
            moveFace(STOP);
            moveStr(STOP);
            if (!reachedGoalOnce) {
                ros::Time goalReached = ros::Time::now();
                ros::Duration timeGoal = goalReached - begin;
                double timeGoalSec = timeGoal.toSec();
                std::cout << "=========================== Reached Goal Coordinate ===========================" << std::endl;
                std::cout << "Time Taken: " << timeGoalSec << "s" << std::endl;
                reachedGoalOnce = true;
            }
        } else {
            if (checkBotReached(posX,posY,nextCoord)) { //have reached next step. Get next coord.
                curCoord = getCoord(posX, posY);
                if (ALGO == 1) {
                    nextCoord = algo.aStar(curCoord); //use aStar algo to get the nextCoord
                } else if (ALGO == 0) {
                    // For FF
                    nextCoord = algo.floodFill(curCoord);
                    if(myRound(posX)!=myRound(initX) || myRound(posY)!=myRound(initY)) 
                        algo.setLastPosition(myRound(initX), myRound(initY));
                }
                std::cout << " ============ Got Next Coord ============" << std::endl;
                movedFlag = true;
                preemptedWall = false;
                numMoves++;
            }
            nextStep = checkCoordFace(curCoord, nextCoord);
            // std::cout << "Next Coord: " << nextCoord.first << " , " << nextCoord.second << std::endl;
            // std::cout << "Next Step: " << nextStep << std::endl;

            if (!checkFace(nextStep)) {
                // std::cout << "Move to Correct Face" << std::endl;
                moveStr(STOP);
                moveFace(nextStep);
            } else if (!checkObs()) {
                //clear path
                // std::cout << "Path Clear" << std::endl;
                moveStr(nextStep);
                // if (!preemptedWall && movedDistance(nextStep, 0.5) && numMoves != 0) {
                if (!preemptedWall && movedDistance(nextStep, 0.5)) {
                    preemptWall(nextStep); //update wall pre-emptively while moving between coordinates.
                    preemptedWall = true; //only run once
                }
            } else {
                std::cout << "=========================== Path Blocked ===========================" << std::endl;
                if (checkBotReached(posX,posY,nextCoord)){ //if reached
                    moveStr(STOP);
                } else {
                    moveStr(nextStep); // continue moving to the coordinate
                }
                curCoord = getCoord(posX, posY);
                algo.updateWall(curCoord, nextStep, true); //update astar algo with wall
                if(ALGO == 1) {
                    nextCoord = algo.aStar(curCoord); // get updated nextCoord
                } else if (ALGO == 0) {
                    // For FF
                    nextCoord = algo.floodFill(curCoord); // get updated nextCoord
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bot_control");
    ros::NodeHandle nh;
    BotController bc(nh);
    ros::spin();
    return 0;
}