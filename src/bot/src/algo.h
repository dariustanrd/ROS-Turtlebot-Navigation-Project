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
    bool operator() (const coord i, const coord j) const{
        if (grid[i.first][i.second].f < grid[j.first][j.second].f) return i < j;
        return i < j;
    }
};

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
    std::set<coord, comp> openSet; //TODO: check custom comparator sort by lowest f FIXME: this doesnt work.
    std::set<coord> closeSet;
    
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
                    grid[i][j].neighbourUp = std::make_pair(-1, -1);
                //Bottom OB
                if (i == 0)
                    grid[i][j].neighbourDown = std::make_pair(-1, -1);
                //Left OB
                if (j == numCol-1)
                    grid[i][j].neighbourLeft = std::make_pair(-1, -1);
                //Right OB
                if (j == 0) 
                    grid[i][j].neighbourRight = std::make_pair(-1, -1);

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

    //TODO: Code aStar algo
    coord aStar(coord pointCoord){
        //full algo
        coord currentCoord;
        coord tempCoord; //FIXME: temp coord for path retrace
        openSet.insert(pointCoord);

        grid[pointCoord.first][pointCoord.second].g = 0;
        grid[pointCoord.first][pointCoord.second].h = manhattanDist(pointCoord, goalCoord);
        grid[pointCoord.first][pointCoord.second].f = grid[pointCoord.first][pointCoord.second].h + grid[pointCoord.first][pointCoord.second].g;
        while(!openSet.empty()) {
            
            // currentCoord = lowest value f in openset;
            std::set<coord>::iterator it = openSet.begin();
            currentCoord = std::make_pair(it->first, it->second);
            
            if (currentCoord == goalCoord) return tempCoord; // - retrace path back to pointCoord, give next coord;

            openSet.erase(openSet.find(currentCoord)); // remove currentCoord from openSet
            closeSet.insert(currentCoord);

            cell spot = grid[currentCoord.first][currentCoord.second];

            // for all neighbours
            // Up neighbour
            if(!closeSet.count(spot.neighbourUp) && !spot.wallUp) {
                int temp = spot.g + manhattanDist(currentCoord, spot.neighbourUp);
                bool newPath = false;
                if (openSet.count(spot.neighbourUp)) {
                    if (temp < grid[spot.neighbourUp.first][spot.neighbourUp.second].g) {
                        grid[spot.neighbourUp.first][spot.neighbourUp.second].g = temp;
                        newPath = true;
                    }
                } else {
                    grid[spot.neighbourUp.first][spot.neighbourUp.second].g = temp;
                    newPath = true;
                    openSet.insert(spot.neighbourUp);
                }
                if (newPath) {
                    grid[spot.neighbourUp.first][spot.neighbourUp.second].h = manhattanDist(spot.neighbourUp, goalCoord);
                    grid[spot.neighbourUp.first][spot.neighbourUp.second].f = grid[spot.neighbourUp.first][spot.neighbourUp.second].g + grid[spot.neighbourUp.first][spot.neighbourUp.second].h;
                    grid[spot.neighbourUp.first][spot.neighbourUp.second].previous = currentCoord;
                }
            }
        }
    }

};