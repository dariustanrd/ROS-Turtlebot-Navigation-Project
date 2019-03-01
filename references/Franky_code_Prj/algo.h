
//************************************************************************************************************************************//
// Search algo file. For generating next grid coord
// Edited by: Darius Tan
//************************************************************************************************************************************//
#include <deque>
#include <queue>
#include <set>
#include <unordered_set>
#include <boost/unordered_set.hpp>
// #include <boost/functional/hash.hpp>
#include <math.h>
// #include <iomanip>
// #include <iostream>

typedef std::pair<int,int> coord;

#define STOP 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4


// TODO: Can try to use a 90 x 90 grid instead and move at 0.1m each time.
// Then allow for diagonal movement? How to make smoothing of turns?

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

    coord gridUp, gridDown, gridRight, gridLeft;

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


void setCoordinate(coord* co, int x=0, int y=0){
	co->first = x;
	co->second = y;
}


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

// This function is to test the moving grid motion for World 1.
// coord pathfinder(coord curCoord, coord goalCoord) {
//     coord nextCoord;
//     //Hardcoding to goal
//     if (curCoord.first == 0 && curCoord.second == 0)
//         nextCoord = std::make_pair(0,1);
//     else if (curCoord.first == 0 && curCoord.second == 1)
//         nextCoord = std::make_pair(1,1);
//     else if (curCoord.first == 1 && curCoord.second == 1)
//         nextCoord = std::make_pair(1,2);
//     else if (curCoord.first == 1 && curCoord.second == 2)
//         nextCoord = std::make_pair(2,2);
//     else if (curCoord.first == 2 && curCoord.second == 2)
//         nextCoord = std::make_pair(3,2);
//     else if (curCoord.first == 3 && curCoord.second == 2)
//         nextCoord = std::make_pair(3,3);
//     else if (curCoord.first == 3 && curCoord.second == 3)
//         nextCoord = std::make_pair(3,4);
//     else if (curCoord.first == 3 && curCoord.second == 4)
//         nextCoord = std::make_pair(4,4);
//     return nextCoord;
// }



// int manhattanDist(coord start, coord end){
//     int horiz = abs(start.first - end.first);
//     int vert = abs(start.second - end.second);
//     return (horiz + vert);
// }

class pathfinderAlgo{
  private:
    std::deque<coord> openSet;
    std::deque<coord> closeSet;
    // boost::unordered_set<coord> openSet;
    // boost::unordered_set<coord> closeSet;
  public:
    // Constructor
    coord lastPosition;




    // pathfinderAlgo() {
    //     closeSet.clear();
    //     openSet.clear();
    //     //create grid
    //     for (int i = 0; i < numRow; ++i) {
    //         for (int j = 0; j < numCol; ++j) {
    //             // Row - i - X
    //             // Col - j - Y
    //             grid[i][j].pos = std::make_pair(i,j);
    //
    //             if (i == numRow - 1)
    //                 grid[i][j].wallRight = true; //right of cell is wall
    //             if (i == 0)
    //                 grid[i][j].wallLeft = true; //left of cell is wall
    //             if (j == numCol-1)
    //                 grid[i][j].wallUp = true; //up of cell is wall
    //             if (j == 0)
    //                 grid[i][j].wallDown = true; //bottom of cell is wall
    //
    //             // Neighbours of that cell
    //             grid[i][j].neighbourUp = std::make_pair(i, j + 1);
    //             grid[i][j].neighbourDown = std::make_pair(i, j - 1);
    //             grid[i][j].neighbourLeft = std::make_pair(i - 1, j);
    //             grid[i][j].neighbourRight = std::make_pair(i + 1, j);
    //
    //             //Right OB
    //             if (i == numRow-1)
    //                 grid[i][j].neighbourRight = OBCoord;
    //             //Left OB
    //             if (i == 0)
    //                 grid[i][j].neighbourLeft = OBCoord;
    //             //Up OB
    //             if (j == numCol-1)
    //                 grid[i][j].neighbourUp = OBCoord;
    //             //Down OB
    //             if (j == 0)
    //                 grid[i][j].neighbourDown = OBCoord;
    //
    //             grid[i][j].previous = std::make_pair(0, 0);
    //
    //             //FIXME: check g/f value comparison
    //             grid[i][j].g = 1;
    //             grid[i][j].h = manhattanDist(grid[i][j].pos, goalCoord);
    //             grid[i][j].f = grid[i][j].g + grid[i][j].h;
    //         }
    //     }
    // }


    void setLastPosition(int x, int y){
    	setCoordinate(&lastPosition, x, y);
    }



    pathfinderAlgo(){
    	pathfinderAlgo(9, 9, 4, 4);
    }

    pathfinderAlgo(int row, int col, int goalRow, int goalCol){
    	numRow = row;
    	numCol = col;
    	closeSet.clear();
    	openSet.clear();
    	for(int i=0; i<numRow; i++){
    		for(int j=0; j<numCol; j++){
    			grid[i][j].wallUp = (i==row-1) ? true : false;
    			grid[i][j].wallDown = (i==0) ? true : false;
    			grid[i][j].wallRight = (j==col-1) ? true : false;
    			grid[i][j].wallLeft = (j==0) ? true : false;
    			//main 4
    			setCoordinate(&grid[i][j].gridUp, i+1, j);
    			setCoordinate(&grid[i][j].gridDown, i-1, j);
    			setCoordinate(&grid[i][j].gridRight, i, j+1);
    			setCoordinate(&grid[i][j].gridLeft, i, j-1);
    			//other 4
    			// setCoordinate(&grid[i][j].gridNW, i+1, j-1);
    			// setCoordinate(&grid[i][j].gridNE, i+1, j+1);
    			// setCoordinate(&grid[i][j].gridSW, i-1, j-1);
    			// setCoordinate(&grid[i][j].gridSE, i-1, j+1);

    			setCoordinate(&grid[i][j].previous);
    			grid[i][j].value = 555;
    			grid[i][j].g = 555;
    		}
    	}
    	setCoordinate(&goalCoord, 4, 4);
    	setCoordinate(&lastPosition, -1, -1);
    }

    // pathfinderAlgo(){
    //   numRow = 9;
    //   numCol = 9;
    //   closeSet.clear();
    //   openSet.clear();
    //   for(int i=0; i<numRow; i++){
    //     for(int j=0; j<numCol; j++){
    //       grid[i][j].wallUp = (i==9-1) ? true : false;
    //       grid[i][j].wallDown = (i==0) ? true : false;
    //       grid[i][j].wallRight = (j==9-1) ? true : false;
    //       grid[i][j].wallLeft = (j==0) ? true : false;
    //       //main 4
    //       setCoordinate(&grid[i][j].gridUp, i+1, j);
    //       setCoordinate(&grid[i][j].gridDown, i-1, j);
    //       setCoordinate(&grid[i][j].gridRight, i, j+1);
    //       setCoordinate(&grid[i][j].gridLeft, i, j-1);
    //       //other 4
    //       // setCoordinate(&grid[i][j].gridNW, i+1, j-1);
    //       // setCoordinate(&grid[i][j].gridNE, i+1, j+1);
    //       // setCoordinate(&grid[i][j].gridSW, i-1, j-1);
    //       // setCoordinate(&grid[i][j].gridSE, i-1, j+1);
    //
    //       setCoordinate(&grid[i][j].previous);
    //       grid[i][j].value = 555;
    //       grid[i][j].g = 555;
    //     }
    //   }
    //   setCoordinate(&goalCoord, GOAL_ROW_X, GOAL_COL_Y);
    //   setCoordinate(&lastPosition, -1, -1);
    // }

    // MazeSolver(int row, int col, int goalRow, int goalCol){
    // 	numRow = row;
    // 	numberOfCols = col;
    // 	close.clear();
    // 	open.clear();
    // 	for(int i=0; i<numberOfRows; i++){
    // 		for(int j=0; j<numberOfCols; j++){
    // 			gridMatrix[i][j].wallUp = (i==row-1) ? true : false;
    // 			gridMatrix[i][j].wallDown = (i==0) ? true : false;
    // 			gridMatrix[i][j].wallRight = (j==col-1) ? true : false;
    // 			gridMatrix[i][j].wallLeft = (j==0) ? true : false;
    // 			//main 4
    // 			setCoordinate(&gridMatrix[i][j].gridUp, i+1, j);
    // 			setCoordinate(&gridMatrix[i][j].gridDown, i-1, j);
    // 			setCoordinate(&gridMatrix[i][j].gridRight, i, j+1);
    // 			setCoordinate(&gridMatrix[i][j].gridLeft, i, j-1);
    // 			//other 4
    // 			setCoordinate(&gridMatrix[i][j].gridNW, i+1, j-1);
    // 			setCoordinate(&gridMatrix[i][j].gridNE, i+1, j+1);
    // 			setCoordinate(&gridMatrix[i][j].gridSW, i-1, j-1);
    // 			setCoordinate(&gridMatrix[i][j].gridSE, i-1, j+1);
    //
    // 			setCoordinate(&gridMatrix[i][j].parent);
    // 			gridMatrix[i][j].value = 555;
    // 			gridMatrix[i][j].g = 555;
    // 		}
    // 	}
    // 	setCoordinate(&goalPosition, goalRow, goalCol);
    // 	setCoordinate(&lastPosition, -1, -1);
    // }




    void resetGrid(){
    	closeSet.clear();
    	// close.push_back(lastPosition);
    	openSet.clear();
    	for(int i=0; i<numRow; i++){
    		for(int j=0; j<numCol; j++){
    			setCoordinate(&grid[i][j].previous);
    			grid[i][j].value = 555;
    			grid[i][j].g = 555;
    		}
    	}
    }
    // void resetGrid() {
    //     closeSet.clear();
    //     openSet.clear();
    //     for (int i = 0; i < numRow; ++i) {
    //         for (int j = 0; j < numCol; ++j) {
    //             grid[i][j].previous = std::make_pair(0, 0);
    //
    //             //FIXME: check g/f value comparison
    //             grid[i][j].g = 1;
    //             grid[i][j].h = manhattanDist(grid[i][j].pos, goalCoord);
    //             grid[i][j].f = grid[i][j].g + grid[i][j].h;
    //         }
    //     }
    // }



    void updateSurroundingWall(coord focusCoordinate){
    	cell focus = grid[focusCoordinate.first][focusCoordinate.second];

    	cell &focusUp = grid[focus.gridUp.first][focus.gridUp.second];
    	if (focusUp.wallDown == false && focusUp.value == 555) focusUp.value = focus.value + 1;

    	cell &focusDown = grid[focus.gridDown.first][focus.gridDown.second];
    	if (focusDown.wallUp == false && focusDown.value == 555) focusDown.value = focus.value + 1;

    	cell &focusRight = grid[focus.gridRight.first][focus.gridRight.second];
    	if (focusRight.wallLeft == false && focusRight.value == 555) focusRight.value = focus.value + 1;

    	cell &focusLeft = grid[focus.gridLeft.first][focus.gridLeft.second];
    	if (focusLeft.wallRight == false && focusLeft.value == 555) focusLeft.value = focus.value + 1;
    }

    void updateWall(coord curCoord, int wallDirection, bool hasWall){
        //update grid with wall
        int row = curCoord.first; //x
        int col = curCoord.second; //y

        switch (wallDirection){
    		case UP:
                if (curCoord.second == GRID_COL_Y_MAX-1) { // at boundary
                    break;
                }
                grid[row][col].wallUp = hasWall;
                grid[row][col+1].wallDown = hasWall;
    		break;
    		case DOWN:
                if (curCoord.second == 0) { // at boundary
                    break;
                }
                grid[row][col].wallDown = hasWall;
                grid[row][col-1].wallUp = hasWall;
    		break;
            case LEFT:
                if (curCoord.first == 0) { // at boundary
                    break;
                }
                grid[row][col].wallLeft = hasWall;
                grid[row-1][col].wallRight = hasWall;
    		break;
    		case RIGHT:
                if (curCoord.first == GRID_ROW_X_MAX-1) { // at boundary
                    break;
                }
                grid[row][col].wallRight = hasWall;
                grid[row+1][col].wallLeft = hasWall;
    		break;
	    }
        std::cout << "Updated walls" << std::endl;
    }



    // coord retrace(coord point) {
    //     coord next, parent;
    //     next = goalCoord;
    //     parent = goalCoord;
    //     while (parent.first != point.first || parent.second != point.second) {
    //         next = parent;
    //         parent = grid[next.first][next.second].parent;
    //     }
    //     return next;
    // }



    // coord lowestF() {
    //     coord lowest;
    //     int temp = 1000;
    //     for (auto it = openSet.begin(); it != openSet.end(); ++it) {
    //         if (grid[it->first][it->second].f < temp) {
    //             temp = grid[it->first][it->second].f;
    //             lowest = *it;
    //         }
    //     }
    //     return lowest;
    // }




    // coord floodfill(coord pointCoord){
    //     std::cout << "Start Floodfill" <<std::endl;
    //     coord currentCoord, nextCoord;
    //     openSet.insert(pointCoord);
    //     std::cout << "Inserted pointCoord to openSet: " << pointCoord.first << " " << pointCoord.second << std::endl;
    // }



    coord floodFill(coord pointCoord){
      int startX = pointCoord.first;
      int startY = pointCoord.second;

      //setup grid value for goal = 0
    	grid[goalCoord.first][goalCoord.second].value = 0;

      //a stack, push goal coordinates to the back of the stack
    	openSet.push_back(goalCoord);

    	coord start, up, down, right, left;

      while (!openSet.empty()){//stack is not empty
    		cell spot = grid[openSet.front().first][openSet.front().second];

    		up = spot.gridUp;
    		down = spot.gridDown;
    		right = spot.gridRight;
    		left = spot.gridLeft;

      	if(spot.wallUp == false && up.first < 9 && grid[up.first][up.second].value == 555)
    			openSet.push_back(up);
      	if(spot.wallRight == false && right.second < 9 && grid[right.first][right.second].value == 555)
    			openSet.push_back(right);
      	if(spot.wallDown == false && down.first >= 0 && grid[down.first][down.second].value == 555)
    			openSet.push_back(down);
      	if(spot.wallLeft == false && left.second >= 0 && grid[left.first][left.second].value == 555)
    			openSet.push_back(left);
      	updateSurroundingWall(openSet.front());
    		openSet.pop_front();
    	}


    	grid[lastPosition.first][lastPosition.second].value = 100;
    	setCoordinate(&start, startX, startY);
    	// return nextGrid(start);

      coord next;//, current;
      next = start;
      cell focus = grid[start.first][start.second];
      int lowest = 555;
      int verdict = 0;

      //check least value only - choose up, right, down, left in that order
      //check left
      if(start.second - 1 >= 0 && !focus.wallLeft && grid[start.first][start.second - 1].value <= lowest){ //gridMatrix[next.x][next.y].value){
        setCoordinate(&next, start.first, start.second-1);
        lowest = grid[start.first][start.second - 1].value;
        verdict = 4;
      }
      //check down
      if(start.first - 1 >= 0 && !focus.wallDown && grid[start.first-1][start.second].value <= lowest){ //gridMatrix[next.x][next.y].value){
        setCoordinate(&next, start.first-1, start.second);
        lowest = grid[start.first-1][start.second].value;
        verdict = 3;
      }
      //check right
      if(start.second + 1 < numCol && !focus.wallRight && grid[start.first][start.second+1].value <= lowest){ //gridMatrix[next.x][next.y].value){
        setCoordinate(&next, start.first, start.second+1);
        lowest = grid[start.first][start.second + 1].value;
        verdict = 2;
      }
      //check up
      if(start.first + 1 < numRow && !focus.wallUp && grid[start.first+1][start.second].value <= lowest){ //gridMatrix[next.x][next.y].value){
        setCoordinate(&next, start.first+1, start.second);
        lowest = grid[start.first+1][start.second].value;
        verdict = 1;
      }
      std::cout<<"verdict: "<<verdict<<std::endl;
      //this part converts "verdict" into next coord for output
      coord nextCoord;
      // return verdict;
      if (verdict == 1){
        nextCoord.first = pointCoord.first;
        nextCoord.second = pointCoord.second + 1;
      }
      if (verdict == 2){
        nextCoord.first = pointCoord.first + 1;
        nextCoord.second = pointCoord.second;
      }
      if (verdict == 3){
        nextCoord.first = pointCoord.first;
        nextCoord.second = pointCoord.second - 1;
      }
      if (verdict == 4){
        nextCoord.first = pointCoord.first - 1;
        nextCoord.second = pointCoord.second;
      }
      return nextCoord;

    }


    // coord aStar(coord pointCoord){
    //     //full algo
    //     // std::cout << "Start AStar" << std::endl;
    //     coord currentCoord, nextCoord;
    //     openSet.insert(pointCoord);
    //     // std::cout << "Inserted pointCoord to openSet: " << pointCoord.first << " " << pointCoord.second << std::endl;
    //
    //     grid[pointCoord.first][pointCoord.second].g = 0;
    //     grid[pointCoord.first][pointCoord.second].h = manhattanDist(pointCoord, goalCoord);
    //     grid[pointCoord.first][pointCoord.second].f = grid[pointCoord.first][pointCoord.second].h + grid[pointCoord.first][pointCoord.second].g;
    //
    //
    //     while(!openSet.empty()) {
    //         // std::cout << "In while openSet empty loop" << std::endl;
    //
    //         // currentCoord = lowest value f in openset
    //         currentCoord = lowestF();
    //         // std::cout << "Current coord with lowest f: " << currentCoord.first << " " << currentCoord.second << std::endl;
    //
    //         if (currentCoord == goalCoord) {
    //             // std::cout << "Found Path" << std::endl;
    //             nextCoord = retrace(pointCoord);
    //             // std::cout << "Next Coord: " << nextCoord.first << " " << nextCoord.second << std::endl;
    //             return nextCoord; // - retrace path back to pointCoord, give next coord;
    //         }
    //
    //         openSet.erase(openSet.find(currentCoord)); // remove currentCoord from openSet
    //         // std::cout << "Erased current coord in openSet" << std::endl;
    //         closeSet.insert(currentCoord);
    //         // std::cout << "Insert current coord in closeSet" << std::endl;
    //
    //         cell spot = grid[currentCoord.first][currentCoord.second];
    //
    //         coord neighbours[4];
    //         neighbours[0] = spot.neighbourUp;
    //         neighbours[1] = spot.neighbourDown;
    //         neighbours[2] = spot.neighbourLeft;
    //         neighbours[3] = spot.neighbourRight;
    //         for (int i = 0; i < 4; ++i) {
    //             // std::cout << "In for neighbours loop: " << i << std::endl;
    //             if ((i == 0) && ((spot.wallUp) || (spot.neighbourUp == OBCoord))) {
    //                 // std::cout << "Invalid UP" << std::endl;
    //                 continue;
    //             }
    //             if ((i == 1) && ((spot.wallDown) || (spot.neighbourDown == OBCoord))) {
    //                 // std::cout << "Invalid DOWN" << std::endl;
    //                 continue;
    //             }
    //             if ((i == 2) && ((spot.wallLeft) || (spot.neighbourLeft == OBCoord))) {
    //                 // std::cout << "Invalid LEFT" << std::endl;
    //                 continue;
    //             }
    //             if ((i == 3) && ((spot.wallRight) || (spot.neighbourRight == OBCoord))) {
    //                 // std::cout << "Invalid RIGHT" << std::endl;
    //                 continue;
    //             }
    //             // std::cout << "Current open set: " << std::endl;
    //             for (auto it2 = openSet.begin(); it2 != openSet.end(); ++it2) {
    //                 // std::cout << it2->first << " " << it2->second << std::endl;
    //             }
    //             if(!closeSet.count(neighbours[i])) {
    //                 //FIXME: check g/f value comparison
    //                 int temp = spot.g + manhattanDist(currentCoord, neighbours[i]);
    //                 bool newPath = false;
    //                 // std::cout << "Neighbour coord: " << neighbours[i].first << " " << neighbours[i].second << std::endl;
    //                 if (openSet.find(neighbours[i]) != openSet.end()) {
    //                     // std::cout << "Neighbour found in openSet: " <<  neighbours[i].first << " " << neighbours[i].second  << std::endl;
    //                     //FIXME: check g/f value comparison
    //                     if (temp < grid[neighbours[i].first][neighbours[i].second].g) {
    //                         // std::cout << "Current f score < neighbour g score" << std::endl;
    //                         grid[neighbours[i].first][neighbours[i].second].g = temp;
    //                         newPath = true;
    //                     } else {
    //                         // std::cout << "Current f score > neighbour g score" << std::endl;
    //                     }
    //                 } else {
    //                     // std::cout << "Neighbour not found in openSet" << std::endl;
    //                     grid[neighbours[i].first][neighbours[i].second].g = temp;
    //                     newPath = true;
    //                     openSet.insert(neighbours[i]);
    //                     // std::cout << "Inserted neighbour direction " << i <<" to openSet: " << neighbours[i].first << " " << neighbours[i].second << std::endl;
    //                     // std::cout << "Current open set: " << std::endl;
    //                     for (auto it3 = openSet.begin(); it3 != openSet.end(); ++it3) {
    //                         std::cout << it3->first << " " << it3->second << std::endl;
    //                     }
    //                 }
    //                 if (newPath) {
    //                     grid[neighbours[i].first][neighbours[i].second].h = manhattanDist(neighbours[i], goalCoord);
    //                     grid[neighbours[i].first][neighbours[i].second].f = grid[neighbours[i].first][neighbours[i].second].g + grid[neighbours[i].first][neighbours[i].second].h;
    //                     grid[neighbours[i].first][neighbours[i].second].previous = currentCoord;
    //                 }
    //             }
    //         }
    //     }
    // }





};
