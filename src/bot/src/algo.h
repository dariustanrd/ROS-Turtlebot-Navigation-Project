//************************************************************************************************************************************//
// Search algo file. For generating next grid coord
// Edited by: Darius Tan
//************************************************************************************************************************************//
#include <deque>
#include <queue>
#include <set>
#include <bits/stdc++.h>
// #include <iomanip>
// #include <iostream>

typedef std::pair<int,int> coord;

#define STOP 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4

#define GRID_ROW_X_MAX 9
#define GRID_COL_Y_MAX 9

#define GOAL_ROW_X 4
#define GOAL_COL_Y 4

#define START_X 0
#define START_Y 0

coord goalCoord (GOAL_ROW_X, GOAL_COL_Y);
coord startCoord (START_X,START_Y);
coord OBCoord (-1,-1);

typedef struct{
    bool wallUp;
    bool wallDown;
    bool wallLeft;
    bool wallRight;
    
    coord neighbourUp;
    coord neighbourDown;
    coord neighbourLeft;
    coord neighbourRight;
    int h;
    int g;
    int f;
    coord previous;
} cell;

cell grid[GRID_ROW_X_MAX][GRID_COL_Y_MAX];

struct comp{
    bool operator() (coord i, coord j) const{
        if (grid[i.first][i.second].f < grid[j.first][j.second].f) return true;
        else return false;
    }
};
std::set<coord, comp> openSet;
std::set<coord> closeSet;
// WRT starting position X = 0, Y = 0, yaw = 0
// Goal position = x: 4 y: 4
// Positive pose on map:	Yaw orientation on map:
//  Y						//          0
//  |						//      +       -
//  |						//    +     .      -
//	|						//      +       -
//  ------> X				//      3.14 -3.14

// This function is to test the moving grid motion.
coord pathfinder(coord curCoord, coord goalCoord) {
    coord nextCoord;
    // try hardcoding to goal
    if (curCoord.first == 0 && curCoord.second == 0) 
        nextCoord = std::make_pair(0,1);
    else if (curCoord.first == 0 && curCoord.second == 1) 
        nextCoord = std::make_pair(1,1);
    else if (curCoord.first == 1 && curCoord.second == 1) 
        nextCoord = std::make_pair(1,2);
    else if (curCoord.first == 1 && curCoord.second == 2) 
        nextCoord = std::make_pair(2,2);
    else if (curCoord.first == 2 && curCoord.second == 2) 
        nextCoord = std::make_pair(3,2);
    else if (curCoord.first == 3 && curCoord.second == 2) 
        nextCoord = std::make_pair(3,3);
    else if (curCoord.first == 3 && curCoord.second == 3) 
        nextCoord = std::make_pair(3,4);
    else if (curCoord.first == 3 && curCoord.second == 4) 
        nextCoord = std::make_pair(4,4);
    return nextCoord;
}

int manhattanDist(coord start, coord end){
    int horiz = abs(start.first - end.first);
    int vert = abs(start.second - end.second);
    return (horiz + vert);
}

class pathfinderAlgo{
  private:
    // std::set<coord, comp> openSet; //TODO: check custom comparator sort by lowest f
    // std::set<coord> closeSet;
    
    // coord goalCoord;

  public:
    // Constructor
    pathfinderAlgo() {
        int numRow = GRID_ROW_X_MAX;
        int numCol = GRID_COL_Y_MAX;

        // goalCoord = std::make_pair(GOAL_ROW_X, GOAL_COL_Y);

        closeSet.clear();
        openSet.clear();
        
        //create grid
        for (int i = 0; i < numRow; ++i) {
            for (int j = 0; j < numCol; ++j) {
                // Row - i - X 
                // Col - j - Y

                if (i == numRow-1)
                    grid[i][j].wallRight = true; //right of cell is wall
                if (i == 0)
                    grid[i][j].wallLeft = true; //left of cell is wall
                if (j == numCol-1)
                    grid[i][j].wallUp = true; //up of cell is wall
                if (j == 0)
                    grid[i][j].wallDown = true; //bottom of cell is wall

                // Neighbours of that cell
                grid[i][j].neighbourUp = std::make_pair(i + 1, j);
                grid[i][j].neighbourDown = std::make_pair(i - 1, j);
                grid[i][j].neighbourLeft = std::make_pair(i, j - 1);
                grid[i][j].neighbourRight = std::make_pair(i, j + 1);
                
                //Top OB
                if (i == numRow-1)
                    grid[i][j].neighbourUp = OBCoord;
                //Bottom OB
                if (i == 0)
                    grid[i][j].neighbourDown = OBCoord;
                //Left OB
                if (j == numCol-1)
                    grid[i][j].neighbourLeft = OBCoord;
                //Right OB
                if (j == 0) 
                    grid[i][j].neighbourRight = OBCoord;

                grid[i][j].previous = std::make_pair(0, 0);
            }
        }
    }
    
    void updateWall(coord curCoord, int wallDirection){
        //update grid with wall
        int row = curCoord.first; //x
        int col = curCoord.second; //y

        switch (wallDirection){
		case UP:
            grid[row][col].wallUp = true;
            grid[row + 1][col].wallDown = true;
		break;
		case DOWN:
            grid[row][col].wallDown = true;
            grid[row - 1][col].wallUp = true;
		break;
        case LEFT:
            grid[row][col].wallLeft = true;
            grid[row][col - 1].wallRight = true;
		break;
		case RIGHT:
            grid[row][col].wallRight = true;
            grid[row][col + 1].wallLeft = true;
		break;
	    }
    }
    coord retrace(coord point) {
        coord next, parent;
        next = goalCoord;
        parent = goalCoord;
        while (parent.first != point.first || parent.second != point.second) {
            next = parent;
            parent = grid[next.first][next.second].previous;
        }
        return next;
    }

    //TODO: Code aStar algo
    coord aStar(coord pointCoord){
        //full algo
        std::cout << "Start AStar" << std::endl;
        coord currentCoord, nextCoord;
        openSet.insert(pointCoord);

        grid[pointCoord.first][pointCoord.second].g = 0;
        grid[pointCoord.first][pointCoord.second].h = manhattanDist(pointCoord, goalCoord);
        grid[pointCoord.first][pointCoord.second].f = grid[pointCoord.first][pointCoord.second].h + grid[pointCoord.first][pointCoord.second].g;
        while(!openSet.empty()) {
            
            // currentCoord = lowest value f in openset;
            std::set<coord>::iterator it = openSet.begin();
            currentCoord = std::make_pair(it->first, it->second);
            
            if (currentCoord == goalCoord) {
                std::cout << "Found Path" << std::endl;
                nextCoord = retrace(pointCoord);
                std::cout << "Next Coord: " << nextCoord.first << " " << nextCoord.second << std::endl;
                return nextCoord; // - retrace path back to pointCoord, give next coord;
            }

            openSet.erase(openSet.find(currentCoord)); // remove currentCoord from openSet
            closeSet.insert(currentCoord);

            cell spot = grid[currentCoord.first][currentCoord.second];

            // for all neighbours
            // bool flag[4] = {false};
            // if (spot.neighbourUp != OBCoord)
            //     neighbours[0] = spot.neighbourUp;
            // else flag[0] = true;
            // if (spot.neighbourDown != OBCoord)
            //     neighbours[1] = spot.neighbourDown;
            // else flag[1] = true;
            // if (spot.neighbourLeft != OBCoord)
            //     neighbours[2] = spot.neighbourLeft;
            // else flag[2] = true;
            // if (spot.neighbourRight != OBCoord)
            //     neighbours[3] = spot.neighbourRight;
            // else flag[3] = true;

            //FIXME: I KEEP GETTING SEGMENTATION FAULTS
            
            coord neighbours[4];
            neighbours[0] = spot.neighbourUp;
            neighbours[1] = spot.neighbourDown;
            neighbours[2] = spot.neighbourLeft;
            neighbours[3] = spot.neighbourRight;
            //FIXME: must make sure the spot coord are within the grid! if not will mess up
            for (int i = 0; i < 4; ++i) {
                if (i == 0 && spot.wallUp) continue;
                if (i == 1 && spot.wallDown) continue;
                if (i == 2 && spot.wallLeft) continue;
                if (i == 3 && spot.wallRight) continue;
                if(!closeSet.count(neighbours[i])) {
                    //FIXME: check my g values, the g score is wrong.
                    int temp = spot.g + manhattanDist(currentCoord, neighbours[i]);
                    bool newPath = false;
                    if (openSet.count(neighbours[i])) {
                        if (temp < grid[neighbours[i].first][neighbours[i].second].g) {
                            grid[neighbours[i].first][neighbours[i].second].g = temp;
                            newPath = true;
                        }
                    } else {
                        grid[neighbours[i].first][neighbours[i].second].g = temp;
                        newPath = true;
                        openSet.insert(neighbours[i]);
                    }
                    if (newPath) {
                        grid[neighbours[i].first][neighbours[i].second].h = manhattanDist(neighbours[i], goalCoord);
                        grid[neighbours[i].first][neighbours[i].second].f = grid[neighbours[i].first][neighbours[i].second].g + grid[neighbours[i].first][neighbours[i].second].h;
                        grid[neighbours[i].first][neighbours[i].second].previous = currentCoord;
                    }
                }
            }

            // // Up neighbour
            // if(!closeSet.count(spot.neighbourUp) && !spot.wallUp && spot.neighbourUp != std::make_pair(-1,-1)) {
            //     int temp = spot.g + manhattanDist(currentCoord, spot.neighbourUp);
            //     bool newPath = false;
            //     if (openSet.count(spot.neighbourUp)) {
            //         if (temp < grid[spot.neighbourUp.first][spot.neighbourUp.second].g) {
            //             grid[spot.neighbourUp.first][spot.neighbourUp.second].g = temp;
            //             newPath = true;
            //         }
            //     } else {
            //         grid[spot.neighbourUp.first][spot.neighbourUp.second].g = temp;
            //         newPath = true;
            //         openSet.insert(spot.neighbourUp);
            //     }
            //     if (newPath) {
            //         grid[spot.neighbourUp.first][spot.neighbourUp.second].h = manhattanDist(spot.neighbourUp, goalCoord);
            //         grid[spot.neighbourUp.first][spot.neighbourUp.second].f = grid[spot.neighbourUp.first][spot.neighbourUp.second].g + grid[spot.neighbourUp.first][spot.neighbourUp.second].h;
            //         grid[spot.neighbourUp.first][spot.neighbourUp.second].previous = currentCoord;
            //     }
            // }

            // // Down neighbour
            // if(!closeSet.count(spot.neighbourDown) && !spot.wallDown && spot.neighbourDown != std::make_pair(-1,-1)) {
            //     int temp = spot.g + manhattanDist(currentCoord, spot.neighbourDown);
            //     bool newPath = false;
            //     if (openSet.count(spot.neighbourDown)) {
            //         if (temp < grid[spot.neighbourDown.first][spot.neighbourDown.second].g) {
            //             grid[spot.neighbourDown.first][spot.neighbourDown.second].g = temp;
            //             newPath = true;
            //         }
            //     } else {
            //         grid[spot.neighbourDown.first][spot.neighbourDown.second].g = temp;
            //         newPath = true;
            //         openSet.insert(spot.neighbourDown);
            //     }
            //     if (newPath) {
            //         grid[spot.neighbourDown.first][spot.neighbourDown.second].h = manhattanDist(spot.neighbourDown, goalCoord);
            //         grid[spot.neighbourDown.first][spot.neighbourDown.second].f = grid[spot.neighbourDown.first][spot.neighbourDown.second].g + grid[spot.neighbourDown.first][spot.neighbourDown.second].h;
            //         grid[spot.neighbourDown.first][spot.neighbourDown.second].previous = currentCoord;
            //     }
            // }

            // // Left neighbour
            // if(!closeSet.count(spot.neighbourLeft) && !spot.wallLeft && spot.neighbourLeft != std::make_pair(-1,-1)) {
            //     int temp = spot.g + manhattanDist(currentCoord, spot.neighbourLeft);
            //     bool newPath = false;
            //     if (openSet.count(spot.neighbourLeft)) {
            //         if (temp < grid[spot.neighbourLeft.first][spot.neighbourLeft.second].g) {
            //             grid[spot.neighbourLeft.first][spot.neighbourLeft.second].g = temp;
            //             newPath = true;
            //         }
            //     } else {
            //         grid[spot.neighbourLeft.first][spot.neighbourLeft.second].g = temp;
            //         newPath = true;
            //         openSet.insert(spot.neighbourLeft);
            //     }
            //     if (newPath) {
            //         grid[spot.neighbourLeft.first][spot.neighbourLeft.second].h = manhattanDist(spot.neighbourLeft, goalCoord);
            //         grid[spot.neighbourLeft.first][spot.neighbourLeft.second].f = grid[spot.neighbourLeft.first][spot.neighbourLeft.second].g + grid[spot.neighbourLeft.first][spot.neighbourLeft.second].h;
            //         grid[spot.neighbourLeft.first][spot.neighbourLeft.second].previous = currentCoord;
            //     }
            // }

            // // Right neighbour
            // if(!closeSet.count(spot.neighbourRight) && !spot.wallRight && spot.neighbourRight != std::make_pair(-1,-1)) {
            //     int temp = spot.g + manhattanDist(currentCoord, spot.neighbourRight);
            //     bool newPath = false;
            //     if (openSet.count(spot.neighbourRight)) {
            //         if (temp < grid[spot.neighbourRight.first][spot.neighbourRight.second].g) {
            //             grid[spot.neighbourRight.first][spot.neighbourRight.second].g = temp;
            //             newPath = true;
            //         }
            //     } else {
            //         grid[spot.neighbourRight.first][spot.neighbourRight.second].g = temp;
            //         newPath = true;
            //         openSet.insert(spot.neighbourRight);
            //     }
            //     if (newPath) {
            //         grid[spot.neighbourRight.first][spot.neighbourRight.second].h = manhattanDist(spot.neighbourRight, goalCoord);
            //         grid[spot.neighbourRight.first][spot.neighbourRight.second].f = grid[spot.neighbourRight.first][spot.neighbourRight.second].g + grid[spot.neighbourRight.first][spot.neighbourRight.second].h;
            //         grid[spot.neighbourRight.first][spot.neighbourRight.second].previous = currentCoord;
            //     }
            // }

        }
    }

};