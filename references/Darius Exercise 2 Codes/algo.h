//************************************************************************************************************************************//
// Search algo file. For generating next grid coord
// Edited by: Darius Tan
//************************************************************************************************************************************//
#include <deque>
#include <queue>
#include <set>
// #include <iomanip>
// #include <iostream>

typedef std::pair<int,int> coord;

#define STOP 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4

#define GRID_ROW_X_MAX 12
#define GRID_COL_Y_MAX 8

#define GOAL_ROW_X 3
#define GOAL_COL_Y 3
#define EXIT_ROW_X 0
#define EXIT_COL_Y 3

// In this map, the start X and Y cannot be 0,0. It is actually 0,6. 
// Pose in pos_info needs to change to start at 0,6
#define START_X 0
#define START_Y 6

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

// This function is to test the moving grid motion.
coord pathfinder(coord curCoord, coord goalCoord) {
    coord nextCoord;
    // try hardcoding to (5,6) to test obstacle detection.
    // if (curCoord.first == 0 && curCoord.second == 6) 
    //     nextCoord = std::make_pair(1,6);
    // else if (curCoord.first == 1 && curCoord.second == 6) 
    //     nextCoord = std::make_pair(2,6);
    // else if (curCoord.first == 2 && curCoord.second == 6) 
    //     nextCoord = std::make_pair(3,6);
    // else if (curCoord.first == 3 && curCoord.second == 6) 
    //     nextCoord = std::make_pair(4,6);
    // else if (curCoord.first == 4 && curCoord.second == 6) 
    //     nextCoord = std::make_pair(5,6);

    // try hardcoding to (3,3)
    if (curCoord.first == 0 && curCoord.second == 6) 
        nextCoord = std::make_pair(1,6);
    else if (curCoord.first == 1 && curCoord.second == 6) 
        nextCoord = std::make_pair(1,5);
    else if (curCoord.first == 1 && curCoord.second == 5) 
        nextCoord = std::make_pair(2,5);
    else if (curCoord.first == 2 && curCoord.second == 5) 
        nextCoord = std::make_pair(2,4);
    else if (curCoord.first == 2 && curCoord.second == 4) 
        nextCoord = std::make_pair(3,4);
    else if (curCoord.first == 3 && curCoord.second == 4) 
        nextCoord = std::make_pair(3,3);
    return nextCoord;
}

typedef struct{
    // bool wallUp;
    // bool wallDown;
    // bool wallLeft;
    // bool wallRight;
    bool wall;
    coord neighbourUp;
    coord neighbourDown;
    coord neighbourLeft;
    coord neighbourRight;
    int value; //h
    int g;
    // int f;
    coord previous;
} cell;

int manhattanDist(coord start, coord end){
    int horiz = abs(start.first - end.first);
    int vert = abs(start.second - end.second);
    return (horiz + vert);
}

class pathfinderAlgo{
  private:
    std::set<coord> openSet; //TODO: implement custom comparator sort by lowest f
    std::set<coord> closeSet;
    cell grid[GRID_ROW_X_MAX][GRID_COL_Y_MAX];
    coord goalCoord;
    coord entranceCoord, exitCoord;

  public:
    // Constructor
    pathfinderAlgo() {
        int numRow = GRID_ROW_X_MAX;
        int numCol = GRID_COL_Y_MAX;

        entranceCoord = std::make_pair(START_X, START_Y);
        exitCoord = std::make_pair(EXIT_ROW_X, EXIT_COL_Y);
        goalCoord = std::make_pair(GOAL_ROW_X, GOAL_COL_Y);

        closeSet.clear();
        openSet.clear();
        
        //create grid
        for (int i = 0; i <= numRow; ++i) {
            for (int j = 0; j <= numCol; ++j) {
                // Row - i - X 
                // Col - j - Y
                
                // Coord is a wall
                if ((i == numRow) || (i == 0) || (j == numCol) || (j == 0)) {
                    grid[i][j].wall = true;
                } else
                    grid[i][j].wall = false;

                // Neighbours of that cell
                grid[i][j].neighbourUp = std::make_pair(i + 1, j);
                grid[i][j].neighbourDown = std::make_pair(i - 1, j);
                grid[i][j].neighbourLeft = std::make_pair(i, j - 1);
                grid[i][j].neighbourRight = std::make_pair(i, j + 1);
                
                //Top OB
                if (i == numRow)
                    grid[i][j].neighbourUp = std::make_pair(-1, -1);
                //Bottom OB
                if (i == 0)
                    grid[i][j].neighbourDown = std::make_pair(-1, -1);
                //Left OB
                if (j == numCol)
                    grid[i][j].neighbourLeft = std::make_pair(-1, -1);
                //Right OB
                if (j == 0) 
                    grid[i][j].neighbourRight = std::make_pair(-1, -1);

                grid[i][j].previous = std::make_pair(0, 0);
            }
        }
        grid[entranceCoord.first][entranceCoord.second].wall = false;
        grid[exitCoord.first][exitCoord.second].wall = false;
    }
    
    void updateWall(coord curCoord, int wallDirection){
        //update grid with wall
        int row = curCoord.first; //x
        int col = curCoord.second; //y

        switch (wallDirection){
		case UP:
            grid[row + 1][col].wall = true;
		break;
		case DOWN:
            grid[row - 1][col].wall = true;
		break;
        case LEFT:
            grid[row][col + 1].wall = true;
		break;
		case RIGHT:
            grid[row][col - 1].wall = true;
		break;
	    }
    }

    //TODO: Code aStar algo
    void aStar(coord startCoord){
        //full algo
        // coord currentCoord;
        // openSet.insert(startCoord);

        // grid[startCoord.first][startCoord.second].g = 0;
        // grid[startCoord.first][startCoord.second].value = manhattanDist(startCoord, goalCoord);
        // while(!openSet.empty()) {
            
            // currentCoord = lowest value f in openset;

            // if (currentCoord == goalCoord) return;

            // openSet.erase(openSet.find(currentCoord)); // remove currentCoord from openSet
            // closeSet.insert(currentCoord);

            // cell spot = grid[currentCoord.first][currentCoord.second];

            //for all neighbours
            // if(!closeSet.count(neighbour) && !neighbour.wall) {
            //     int temp = spot.g + manhattanDist(currentCoord, nbhCoord);
            //     bool newPath = false;
            //     if (openSet.count(nbhCoord)) {
            //         if (temp < neighbour.g) {
            //             neighbour.g = temp;
            //             newPath = true;
            //         }
            //     } else {
            //         neighbour.g = temp;
            //         newPath = true;
            //         openSet.insert(nbhCoord);
            //     }
            //     if (newPath) {
            //         neighbour.h = manhattanDist(nbhCoord, goalCoord);
            //         neighbour.f = neighbour.g + neighbour.h;
            //         neighbour.previous = spot;
            //     }
            // }
        // }
    }

};