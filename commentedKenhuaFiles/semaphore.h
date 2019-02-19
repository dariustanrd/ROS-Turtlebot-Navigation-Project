/* This is another custom written file to supplement the turtlebot navigation.
 * It is a semaphore to limit the robot on only moving in one direction.
 * Written by Liew Ken Hua.
 */
class Semaphore{
private:
	bool up;
	bool down;
	bool right;
	bool left;
public:
	void setUp();
	void setDown();
	void setRight();
	void setLeft();
	void setSemaphore(int direction);
	bool isUp();
	bool isDown();
	bool isRight();
	bool isLeft();
	bool checkIfItIs(int direction);
};


//###########################################################################
//sets of functions to set direction boolean
//if set to Up, then up boolean is true etc

void Semaphore::setUp(){
	up = true;
	down = false;
	right = false;
	left = false;
}
void Semaphore::setDown(){
	up = false;
	down = true;
	right = false;
	left = false;
}
void Semaphore::setRight(){
	up = false;
	down = false;
	right = true;
	left = false;
}
void Semaphore::setLeft(){
	up = false;
	down = false;
	right = false;
	left = true;
}

//############################################################################
//switch cases using setSemaphore function, with input of direction number
//input direction integer==>set direction boolean accordingly

void Semaphore::setSemaphore(int direction){
	switch(direction){
		case 1:
			setUp();
			break;
		case 2:
			setRight();
			break;
		case 3:
			setDown();
			break;
		case 4:
			setLeft();
			break;
		default:
			setUp();
	}
}

//##########################################################################
//sets of functions, return current boolean values
// i.e if "is Up" is called, function will return up's current status
bool Semaphore::isUp(){
	return up;
}
bool Semaphore::isDown(){
	return down;
}
bool Semaphore::isRight(){
	return right;
}
bool Semaphore::isLeft(){
	return left;
}

//############################################################################
//switch cases function to check direction status
// if direction number is inputted, function will return the status
// i.e. if 1 is inputted, it'll call isUp function, then isUp function will return up's current boolean status
bool Semaphore::checkIfItIs(int direction){
	switch(direction){
		case 1:
			return isUp();
		case 2:
			return isRight();
		case 3:
			return isDown();
		case 4:
			return isLeft();
		default:
			return false;
	}
}
