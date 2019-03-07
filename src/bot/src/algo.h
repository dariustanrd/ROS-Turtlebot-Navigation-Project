//************************************************************************************************************************************//
// Search algo file. For generating next grid coord
// Edited by: Darius Tan & Franky Laurentis
//************************************************************************************************************************************//
#include <deque>
#include <set>
#include <boost/unordered_set.hpp>
#include <math.h>

typedef std::pair<int,int> coord;

//*********************************************************//
//**************** DEFINE ALGO CHOICE HERE ****************//
// ALGO 0 - Flood Fill
// ALGO 1 - A*

#define ALGO 1
//*********************************************************//

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
    coord pos;

    bool wallUp;
    bool wallDown;
    bool wallLeft;
    bool wallRight;

    // For FF
    int value;

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

int numRow = GRID_ROW_X_MAX;
int numCol = GRID_COL_Y_MAX;

// WRT starting position X = 0, Y = 0, yaw = 0
// Goal position = x: 4 y: 4
// Positive pose on map:	Yaw orientation on map:
//  Y						//          0
//  |						//      +       -
//  |						//    +     .      -
//	|						//      +       -
//  ------> X				//      3.14 -3.14

int manhattanDist(coord start, coord end){
    int horiz = abs(start.first - end.first);
    int vert = abs(start.second - end.second);
    return (horiz + vert);
}

class pathfinderAlgo{
  private:
    //For FF
    std::deque<coord> stackFF;

    boost::unordered_set<coord> openSet;
    boost::unordered_set<coord> closeSet;
  public:
    // Constructor
    pathfinderAlgo() {
        closeSet.clear();
        openSet.clear();
        //For FF
        stackFF.clear();
        //create grid
        for (int i = 0; i < numRow; ++i) {
            for (int j = 0; j < numCol; ++j) {
                // Row - i - X 
                // Col - j - Y
                grid[i][j].pos = std::make_pair(i,j);

                if (i == numRow - 1)
                    grid[i][j].wallRight = true; //right of cell is wall
                if (i == 0)
                    grid[i][j].wallLeft = true; //left of cell is wall
                if (j == numCol-1)
                    grid[i][j].wallUp = true; //up of cell is wall
                if (j == 0)
                    grid[i][j].wallDown = true; //bottom of cell is wall

                // Neighbours of that cell
                grid[i][j].neighbourUp = std::make_pair(i, j + 1);
                grid[i][j].neighbourDown = std::make_pair(i, j - 1);
                grid[i][j].neighbourLeft = std::make_pair(i - 1, j);
                grid[i][j].neighbourRight = std::make_pair(i + 1, j);
                
                //Right OB
                if (i == numRow-1)
                    grid[i][j].neighbourRight = OBCoord;
                //Left OB
                if (i == 0)
                    grid[i][j].neighbourLeft = OBCoord;
                //Up OB
                if (j == numCol-1)
                    grid[i][j].neighbourUp = OBCoord;
                //Down OB
                if (j == 0) 
                    grid[i][j].neighbourDown = OBCoord;

                grid[i][j].previous = std::make_pair(0, 0);

                if (ALGO == 1) { // aStar algo
                    //g value set as 1 for all grids --> cost of moving between grids is the same
                    grid[i][j].g = 1;
                    grid[i][j].h = manhattanDist(grid[i][j].pos, goalCoord);
                    grid[i][j].f = grid[i][j].g + grid[i][j].h;
                } else if (ALGO == 0) { // FF algo
                    grid[i][j].value = 555;
                    grid[i][j].g = 555;
                    lastPos = OBCoord;
                }
            }
        }
    }
    //For FF
    coord lastPos;
    void setLastPosition(int x, int y){
        lastPos = std::make_pair(x,y);
        std::cout<< "Last Position: "<< lastPos.first << " , " << lastPos.second <<std::endl;
    }
    void resetGrid() {
        closeSet.clear();
        openSet.clear();
        for (int i = 0; i < numRow; ++i) {
            for (int j = 0; j < numCol; ++j) {
                grid[i][j].previous = std::make_pair(0, 0);

                if (ALGO == 1) { // aStar algo
                    //g value set as 1 for all grids --> cost of moving between grids is the same
                    grid[i][j].g = 1;
                    grid[i][j].h = manhattanDist(grid[i][j].pos, goalCoord);
                    grid[i][j].f = grid[i][j].g + grid[i][j].h;
                } else if (ALGO == 0) {  // FF algo
                    grid[i][j].value = 555;
                    grid[i][j].g = 555;
                }
            }
        }
    }
    //For FF
    void updateSurroundingWall(coord focusCoordinate){
    	cell focus = grid[focusCoordinate.first][focusCoordinate.second];

    	cell &focusUp = grid[focus.neighbourUp.first][focus.neighbourUp.second];
    	if (focusUp.wallDown == false && focusUp.value == 555) focusUp.value = focus.value + 1;

    	cell &focusDown = grid[focus.neighbourDown.first][focus.neighbourDown.second];
    	if (focusDown.wallUp == false && focusDown.value == 555) focusDown.value = focus.value + 1;

    	cell &focusRight = grid[focus.neighbourRight.first][focus.neighbourRight.second];
    	if (focusRight.wallLeft == false && focusRight.value == 555) focusRight.value = focus.value + 1;

    	cell &focusLeft = grid[focus.neighbourLeft.first][focus.neighbourLeft.second];
    	if (focusLeft.wallRight == false && focusLeft.value == 555) focusLeft.value = focus.value + 1;
    }

    void updateWall(coord curCoord, int wallDirection, bool hasWall){
        //update grid with wall
        int row = curCoord.first; //x
        int col = curCoord.second; //y

        switch (wallDirection){
		case UP:
            if (curCoord.second == GRID_COL_Y_MAX-1) { //Exception handling for boundary case
                // std::cout << "Boundary Case" << std::endl;
                break;
            }
            grid[row][col].wallUp = hasWall;
            grid[row][col+1].wallDown = hasWall;
		break;
		case DOWN:
            if (curCoord.second == 0) { //Exception handling for boundary case
                // std::cout << "Boundary Case" << std::endl;
                break;
            }
            grid[row][col].wallDown = hasWall;
            grid[row][col-1].wallUp = hasWall;
		break;
        case LEFT:
            if (curCoord.first == 0) { //Exception handling for boundary case
                // std::cout << "Boundary Case" << std::endl;
                break;
            }
            grid[row][col].wallLeft = hasWall;
            grid[row-1][col].wallRight = hasWall;
		break;
		case RIGHT:
            if (curCoord.first == GRID_ROW_X_MAX-1) { //Exception handling for boundary case
                // std::cout << "Boundary Case" << std::endl;
                break;
            }
            grid[row][col].wallRight = hasWall;
            grid[row+1][col].wallLeft = hasWall;
		break;
	    }
        // std::cout << "Updated walls" << std::endl;
    }
    coord retrace(coord point) {
        coord next, parent;
        next = goalCoord;
        parent = goalCoord;
        // std::cout << "Retraced path: " << std::endl;
        while (parent.first != point.first || parent.second != point.second) {
            next = parent;
            parent = grid[next.first][next.second].previous;
            // std::cout << next.first << " , " << next.second << std::endl;
        }
        return next;
    }
    coord lowestF() {
        coord lowest;
        int temp = 1000;
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            if (grid[it->first][it->second].f < temp) {
                temp = grid[it->first][it->second].f;
                lowest = *it;
            }
        }
        return lowest;
    }
    bool checkValid(coord testCoord) {
        return (testCoord.first >= 0) && (testCoord.first < GRID_ROW_X_MAX) 
        && (testCoord.second >= 0) && (testCoord.second < GRID_ROW_X_MAX);
    }

    //For FF
    coord floodFill(coord pointCoord) {
        std::cout << "********* Start Floodfill *********" << std::endl;
        //current position = starting position
        int startX = pointCoord.first;
        int startY = pointCoord.second;

        //setup grid value for goal = 0
        grid[goalCoord.first][goalCoord.second].value = 0;

        //a stack, push goal coordinates to the back of the stack
        stackFF.push_back(goalCoord);

        coord start, up, down, right, left;

        while (!stackFF.empty()) { //stack is not empty
            cell spot = grid[stackFF.front().first][stackFF.front().second];

            up = spot.neighbourUp;
            down = spot.neighbourDown;
            right = spot.neighbourRight;
            left = spot.neighbourLeft;

            if (spot.wallUp == false && up.first < 9 && grid[up.first][up.second].value == 555)
                stackFF.push_back(up);
            if (spot.wallRight == false && right.second < 9 && grid[right.first][right.second].value == 555)
                stackFF.push_back(right);
            if (spot.wallDown == false && down.first >= 0 && grid[down.first][down.second].value == 555)
                stackFF.push_back(down);
            if (spot.wallLeft == false && left.second >= 0 && grid[left.first][left.second].value == 555)
                stackFF.push_back(left);

            updateSurroundingWall(stackFF.front());
            stackFF.pop_front();
        }

        grid[lastPos.first][lastPos.second].value = 100;
        start = std::make_pair(startX, startY);
        // return nextGrid(start);

        coord next; //, current;
        next = start;
        cell focus = grid[start.first][start.second];
        int lowest = 555;
        int verdict = 0;

        //check least value only - choose up, right, down, left in that order
        //check left
        if (start.first - 1 >= 0 && !focus.wallLeft && grid[start.first - 1][start.second].value <= lowest) { //gridMatrix[next.x][next.y].value){
            next = std::make_pair(start.first - 1, start.second);
            lowest = grid[start.first - 1][start.second].value;
            verdict = LEFT;
        }
        //check down
        if (start.second - 1 >= 0 && !focus.wallDown && grid[start.first][start.second - 1].value <= lowest) { //gridMatrix[next.x][next.y].value){
            next = std::make_pair(start.first, start.second - 1);
            lowest = grid[start.first][start.second - 1].value;
            verdict = DOWN;
        }
        //check right
        if (start.first + 1 < numCol && !focus.wallRight && grid[start.first][start.second + 1].value <= lowest) { //gridMatrix[next.x][next.y].value){
            next = std::make_pair(start.first + 1, start.second);
            lowest = grid[start.first + 1][start.second + 1].value;
            verdict = RIGHT;
        }
        //check up
        if (start.second + 1 < numRow && !focus.wallUp && grid[start.first + 1][start.second].value <= lowest) { //gridMatrix[next.x][next.y].value){
            next = std::make_pair(start.first, start.second + 1);
            lowest = grid[start.first][start.second + 1].value;
            verdict = UP;
        }
        std::cout << "verdict: " << verdict << std::endl;
        //this part converts "verdict" into next coord for output
        coord nextCoord;
        switch (verdict) {
            case UP: //go up
                nextCoord.first = pointCoord.first;
                nextCoord.second = pointCoord.second + 1;
                break;
            case RIGHT: //go right
                nextCoord.first = pointCoord.first + 1;
                nextCoord.second = pointCoord.second;
                break;
            case DOWN: //go down
                nextCoord.first = pointCoord.first;
                nextCoord.second = pointCoord.second - 1;
                break;
            case LEFT: //go left
                nextCoord.first = pointCoord.first - 1;
                nextCoord.second = pointCoord.second;
                break;
        }
        return nextCoord;
    }

    coord aStar(coord pointCoord){
        //full algo
        std::cout << "********* Start AStar *********" << std::endl;

        if (!checkValid(pointCoord)) { //Exception handling for boundary case
            std::cout << "********* Invalid coordinate passed to aStar *********" << std::endl;
            return pointCoord;
        }

        if (pointCoord == goalCoord) { //Exception handling
            std::cout << "********* Already at goal *********" << std::endl;
            return pointCoord;
        }
        
        coord currentCoord, nextCoord;
        openSet.insert(pointCoord);
        // std::cout << "Inserted pointCoord to openSet: " << pointCoord.first << " " << pointCoord.second << std::endl;

        grid[pointCoord.first][pointCoord.second].g = 0;
        grid[pointCoord.first][pointCoord.second].h = manhattanDist(pointCoord, goalCoord);
        grid[pointCoord.first][pointCoord.second].f = grid[pointCoord.first][pointCoord.second].h + grid[pointCoord.first][pointCoord.second].g;
        while(!openSet.empty()) {
            // std::cout << "In while openSet empty loop" << std::endl;
            
            // currentCoord = lowest value f in openset
            currentCoord = lowestF();
            // std::cout << "Current coord with lowest f: " << currentCoord.first << " " << currentCoord.second << std::endl;
            
            if (currentCoord == goalCoord) {
                std::cout << "********* Found Path *********" << std::endl;
                nextCoord = retrace(pointCoord);
                std::cout << "Next Coord: " << nextCoord.first << " " << nextCoord.second << std::endl;
                return nextCoord; // - retrace path back to pointCoord, give next coord;
            }

            openSet.erase(openSet.find(currentCoord)); // remove currentCoord from openSet
            // std::cout << "Erased current coord in openSet" << std::endl;
            closeSet.insert(currentCoord);
            // std::cout << "Insert current coord in closeSet" << std::endl;

            cell spot = grid[currentCoord.first][currentCoord.second];
            
            coord neighbours[4];
            neighbours[0] = spot.neighbourUp;
            neighbours[1] = spot.neighbourDown;
            neighbours[2] = spot.neighbourLeft;
            neighbours[3] = spot.neighbourRight;
            for (int i = 0; i < 4; ++i) {
                // std::cout << "In for neighbours loop: " << i << std::endl;
                if ((i == 0) && ((spot.wallUp) || (spot.neighbourUp == OBCoord))) {
                    // std::cout << "Invalid UP" << std::endl;
                    continue;
                }
                if ((i == 1) && ((spot.wallDown) || (spot.neighbourDown == OBCoord))) {
                    // std::cout << "Invalid DOWN" << std::endl;
                    continue;
                }
                if ((i == 2) && ((spot.wallLeft) || (spot.neighbourLeft == OBCoord))) {
                    // std::cout << "Invalid LEFT" << std::endl;
                    continue;
                }
                if ((i == 3) && ((spot.wallRight) || (spot.neighbourRight == OBCoord))) {
                    // std::cout << "Invalid RIGHT" << std::endl;
                    continue;
                }
                // std::cout << "Current open set: " << std::endl;
                // for (auto it2 = openSet.begin(); it2 != openSet.end(); ++it2) {
                //     std::cout << it2->first << " " << it2->second << std::endl;
                // }
                if(!closeSet.count(neighbours[i])) {
                    int temp = spot.g + manhattanDist(currentCoord, neighbours[i]);
                    bool newPath = false;
                    // std::cout << "Neighbour coord: " << neighbours[i].first << " " << neighbours[i].second << std::endl;
                    if (openSet.find(neighbours[i]) != openSet.end()) {
                        // std::cout << "Neighbour found in openSet: " <<  neighbours[i].first << " " << neighbours[i].second  << std::endl;
                        if (temp < grid[neighbours[i].first][neighbours[i].second].g) {
                            // std::cout << "Current f score < neighbour g score" << std::endl;
                            grid[neighbours[i].first][neighbours[i].second].g = temp;
                            newPath = true;
                        } else {
                            // std::cout << "Current f score > neighbour g score" << std::endl;
                        }
                    } else {
                        // std::cout << "Neighbour not found in openSet" << std::endl;
                        grid[neighbours[i].first][neighbours[i].second].g = temp;
                        newPath = true;
                        openSet.insert(neighbours[i]);
                        // std::cout << "Inserted neighbour direction " << i <<" to openSet: " << neighbours[i].first << " " << neighbours[i].second << std::endl;
                        // std::cout << "Current open set: " << std::endl;
                        // for (auto it3 = openSet.begin(); it3 != openSet.end(); ++it3) {
                        //     std::cout << it3->first << " " << it3->second << std::endl;
                        // }
                    }
                    if (newPath) {
                        grid[neighbours[i].first][neighbours[i].second].h = manhattanDist(neighbours[i], goalCoord);
                        grid[neighbours[i].first][neighbours[i].second].f = grid[neighbours[i].first][neighbours[i].second].g + grid[neighbours[i].first][neighbours[i].second].h;
                        grid[neighbours[i].first][neighbours[i].second].previous = currentCoord;
                    }
                }
            }
        }
    }

};