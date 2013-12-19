#pragma config(Sensor, S1, HTEOPD, sensorAnalogActive)
#pragma config(Sensor, S3, lightSensor, sensorLightActive)
#include "drivers/hitechnic-eopd.h";

//Ryan McKay and George Jaray
//header

typedef struct node;
typedef struct objectNode;
typedef struct graphArray;
typedef struct point;

//global variables
const int width = 8;
const int height = 8;
const unsigned char startingX = 0;
const unsigned char startingY = 0;
const int goalX = 7;
const int goalY = 0;
unsigned char xLoc = startingX;
unsigned char yLoc = startingY;
int adjust = 0;
int direction = 0; //0 is north, 1 is east, 2 is south, 3 is west
node *starting;
node *goal;

void turnRight90();
void turnLeft90();
void addObstacle();
void calcAStar();
void calcPath();
void runPath();
int heuristic(int startX, int startY);
void paintScreen();

struct{
	unsigned char x;
	unsigned char y;
}point;
//struct defs
struct{
	unsigned char fx;
	unsigned char gx;
	unsigned char x;
	unsigned char y;
	node* parent;
}node;
struct{
	bool open;
	bool object;
	bool closed;
}objectNode;

struct{
	node graph[height][width];
	objectNode objectGraph[height][width];
}graphArray;



graphArray currentGraph;
point path[50];
int pathLength = 0;

//function defs
void turnLeft90(){
	adjust = 0;
	motor[motorA] = 35;
	motor[motorB] = -35;
	wait1Msec(560);  //595 when 25
	motor[motorA] = 0;
	motor[motorB] =0;
	wait1Msec(150);
	if(direction != 0) direction--;
	else direction = 3;
}
void turnRight90(){
	adjust = 0;
	motor[motorB] = 35;
	motor[motorA] = -35;
	wait1Msec(630);  //625 when 25
	motor[motorA] = 0;
	motor[motorB] =0;
	wait1Msec(150);
	if(direction != 3) direction++;
	else direction = 0;
}
void oneForward(){
	int timer = 0;
	while((HTEOPDreadProcessed(HTEOPD) <= 7) && (timer < 110) && (SensorRaw[lightSensor] >= 450)){
		motor[motorA] = 60;
		motor[motorB] = 60;
		wait1Msec(10);
		timer++;
	}
	//motor[motorA] = 0;
	//motor[motorB] = 0;
	//wait1Msec(250);
	if((HTEOPDreadProcessed(HTEOPD) > 7) || (SensorRaw[lightSensor] < 450)){
		motor[motorA] = 0;
		motor[motorB] =0;
		wait1Msec(100);
		for(int i = 0; i < timer+5; i++){
			motor[motorA] = -60;
			motor[motorB] = -60;
			wait1Msec(10);
		}
		motor[motorA] = 0;
		motor[motorB] =0;
		wait1Msec(250);
		timer = 0;
    addObstacle();

	}
	else{
		int prevX = xLoc;
		int prevY = yLoc;
		if(direction == 0) xLoc++;
		else if(direction ==1 )yLoc++;
		else if(direction ==2 )xLoc--;
		else yLoc--;

		nxtDrawLine(((prevY)*6)+3 , ((prevX)*6)+3, ((yLoc)*6)+3, ((xLoc)*6)+3);

		/*adjust++;
		if(adjust == 2){
			adjust = 0;
			motor[motorA] = -25;
			motor[motorB] = 25;
			wait1Msec(200);
			motor[motorA] = 0;
			motor[motorB]=0;
		}*/
	}


}
void addObstacle(){
	PlaySound(soundBeepBeep);
	if(direction == 0){
		currentGraph.objectGraph[xLoc + 1][yLoc].object = true;
	}
	else if(direction == 1){
		currentGraph.objectGraph[xLoc][yLoc+1].object = true;
	}
	else if(direction == 2){
		currentGraph.objectGraph[xLoc - 1][yLoc].object = true;
	}
	else if(direction == 3){
		currentGraph.objectGraph[xLoc][yLoc -1].object = true;
	}
	calcAStar();
	pathLength = 0;
}
void calcAStar(){
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			currentGraph.graph[i][j].gx = 250;
			currentGraph.graph[i][j].fx = 250;
			currentGraph.graph[i][j].parent = NULL;
			currentGraph.objectGraph[i][j].open = false;
			currentGraph.objectGraph[i][j].closed = false;
		}
	}
	node neighbors[4];
	currentGraph.objectGraph[xLoc][yLoc].open = true;
	currentGraph.graph[xLoc][yLoc].gx = 0;
	currentGraph.graph[xLoc][yLoc].fx = 0;
	node* current;
	current = &currentGraph.graph[xLoc][yLoc];
	int i = xLoc;
	int j = yLoc;
	int neighborSize = 0;
	unsigned char cost = 0;
	while(current != goal){
		cost = current->gx;
		cost++;
		currentGraph.objectGraph[i][j].open = false;
		currentGraph.objectGraph[i][j].closed = true;
		neighborSize = 0;
		//find Neighbors
		if(i-1 >= 0){
			neighbors[neighborSize].gx = currentGraph.graph[i-1][j].gx;
			neighbors[neighborSize].fx = currentGraph.graph[i-1][j].fx;
			neighbors[neighborSize].x = currentGraph.graph[i-1][j].x;
			neighbors[neighborSize].y = currentGraph.graph[i-1][j].y;
			neighborSize++;
		}
		if(i+1 < height){
			neighbors[neighborSize].gx = currentGraph.graph[i+1][j].gx;
			neighbors[neighborSize].fx = currentGraph.graph[i+1][j].fx;
			neighbors[neighborSize].x = currentGraph.graph[i+1][j].x;
			neighbors[neighborSize].y = currentGraph.graph[i+1][j].y;
			neighborSize++;
		}
		if(j+1 < width){
			neighbors[neighborSize].gx = currentGraph.graph[i][j+1].gx;
			neighbors[neighborSize].fx = currentGraph.graph[i][j+1].fx;
			neighbors[neighborSize].x = currentGraph.graph[i][j+1].x;
			neighbors[neighborSize].y = currentGraph.graph[i][j+1].y;
			neighborSize++;
		}
		if(j-1 >=0){
			neighbors[neighborSize].gx = currentGraph.graph[i][j-1].gx;
			neighbors[neighborSize].fx = currentGraph.graph[i][j-1].fx;
			neighbors[neighborSize].x = currentGraph.graph[i][j-1].x;
			neighbors[neighborSize].y = currentGraph.graph[i][j-1].y;
			neighborSize++;
		}
		//calc A* for each neighbor
		for(int k = 0; k < neighborSize; k++){
				unsigned char currentGx = neighbors[k].gx;
				if(currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].open == true && cost < currentGx){
						currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].open = false;
				}
				else if(currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].closed == true && cost < currentGx){
					currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].closed = false;
				}
				else if(currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].open == false && currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].closed == false && currentGraph.objectGraph[neighbors[k].x][neighbors[k].y].object == false){
					unsigned char x = neighbors[k].x;
					unsigned char y = neighbors[k].y;
					currentGraph.graph[x][y].gx = cost;
					unsigned char hx = heuristic((int)neighbors[k].x ,(int)neighbors[k].y);
					currentGraph.graph[(int)neighbors[k].x][(int)neighbors[k].y].fx = cost + hx;
					currentGraph.graph[(int)neighbors[k].x][(int)neighbors[k].y].parent = current;
					currentGraph.objectGraph[(int)neighbors[k].x][(int)neighbors[k].y].open = true;
				}
		}
		unsigned char low = 255;
		for(int w = 0; w < height; w++){
			for(int z = 0; z < width; z++){
				if(currentGraph.objectGraph[w][z].open == true && currentGraph.graph[w][z].fx < low){
						low = currentGraph.graph[w][z].fx;
						current = &currentGraph.graph[w][z];
						i = w;
						j = z;
				}
			}
		}

	}
	calcPath();
}
int heuristic(int startX, int startY){
	int dx = abs(startX - goalX);
	int dy = abs(startY - goalY);
	return 5 * (dx + dy);
}

void calcPath(){
		for(int i = 0; i < 50; i++){
			path[i].x = 0;
			path[i].y = 0;
		}
	  pathLength = 0;
		node* current = goal;
		while(current != NULL){
			current = current->parent;
			pathLength++;
		}
		current = goal;
		int insertLoc = pathLength;
		while(current != NULL){
			path[insertLoc].x = current->x;
			path[insertLoc].y = current->y;
			current = current->parent;
			insertLoc--;
		}
		paintScreen();
		runPath();
}

void runPath(){
	int currentVal = 1;
	bool atGoal = false;
	while(currentVal <= pathLength && atGoal == false){
		if(direction == 0){
				if(path[currentVal].x > xLoc){
					oneForward();
				}
				else if(path[currentVal].y > yLoc){
					turnRight90();
					oneForward();
				}
				else if(path[currentVal].y < yLoc){
					turnLeft90();
					oneForward();
				}
				else if(path[currentVal].x < xLoc){
					turnLeft90();
					turnLeft90();
					oneForward();
				}
		}
		else if(direction == 1){
				if(path[currentVal].x > xLoc){
					turnLeft90();
					oneForward();
				}
				else if(path[currentVal].x < xLoc){
					turnRight90();
					oneForward();
				}
				else if(path[currentVal].y > yLoc){
					oneForward();
				}
				else if(path[currentVal].y < yLoc){
					turnLeft90();
					turnLeft90();
					oneForward();
				}
		}
		else if(direction == 2){
				if(path[currentVal].x < xLoc){
					oneForward();
				}
				else if(path[currentVal].x > xLoc){
					turnLeft90();
					turnLeft90();
					oneForward();
				}
				else if(path[currentVal].y < xLoc){
					turnLeft90();
					oneForward();
				}
				else if(path[currentVal].y > yLoc){
					turnRight90();
					oneForward();
				}
		}
		else if(direction == 3){
			if(path[currentVal].x > xLoc){
				turnRight90();
				oneForward();
			}
			else if(path[currentVal].x < xLoc){
				turnLeft90();
				oneForward();
			}
			else if(path[currentVal].y < yLoc){
				oneForward();
			}
			else if(path[currentVal].y > yLoc){
				turnLeft90();
				turnLeft90();
				oneForward();
			}
		}
		currentVal++;
		if(xLoc == goalX && yLoc == goalY){
			atGoal = true;
		}
	}
}
void paintScreen(){
	//eraseDisplay();

	//draw map
	nxtDrawRect(0,height*6 ,width*6,0);
	//draw start node
	nxtFillEllipse(startingY*6, (startingX+1)*6, (startingY+1)*6, startingX*6);
	//draw end node
	nxtFillRect(goalY*6, (goalX+1)*6, (goalY+1)*6, goalX*6);
	//draw objects
	for(int i= 0; i < height; i++){
		for(int j = 0; j < width; j++){
			if(currentGraph.objectGraph[i][j].object == true){
				nxtDrawRect((j*6), (i+1)*6, (j+1)*6, (i*6));
			}
		}
	}
	//draw path
	/**
	for(int i=1; i < pathLength; i++){
		nxtDrawLine(((path[i].y)*6)+3 , ((path[i].x)*6)+3, ((path[i+1].y)*6)+3, ((path[i+1].x)*6)+3);
	}
	*/
}
task main()
{
	nMotorPIDSpeedCtrl[motorA] = 20;
	nMotorPIDSpeedCtrl[motorB] = 20;
	for(int i = 0; i < 50; i++){
		point filler;
		filler.x = 0;
		filler.y = 0;
		path[i] = filler;
	}
	objectNode currentObject;
	node currentNode;
	currentNode.gx = 200;
	currentNode.fx = 200;
	currentNode.x = 0;
	currentNode.y = 0;
	currentNode.parent = NULL;
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			currentObject.object = false;
			currentObject.closed = false;
			currentGraph.graph[i][j] = currentNode;
			currentGraph.graph[i][j].gx = 200;
			currentGraph.graph[i][j].fx = 200;
			currentGraph.graph[i][j].x = i;
			currentGraph.graph[i][j].y = j;
			currentGraph.objectGraph[i][j] = currentObject;
		}
	}
	// set start node to 0;
	currentGraph.graph[startingX][startingY].gx = 0;
	currentGraph.graph[startingX][startingY].fx = 0;
	// set location of known objects here

	/*currentGraph.objectGraph[2][2].object = true;
	currentGraph.objectGraph[2][4].object = true;
	currentGraph.objectGraph[3][4].object = true;
	currentGraph.objectGraph[4][4].object = true;
	currentGraph.objectGraph[5][4].object = true;
	currentGraph.objectGraph[2][5].object = true;
	currentGraph.objectGraph[2][6].object = true;
	currentGraph.objectGraph[0][4].object = true;
*/
	starting = &currentGraph.graph[startingX][startingY];
	goal = &currentGraph.graph[goalX][goalY];
	calcAStar();
	motor[motorA] = 0;
	motor[motorB] = 0;
	int time = nPgmTime/1000;
	PlaySound(soundFastUpwardTones);
	PlaySound(soundDownwardTones);
	PlaySound(soundFastUpwardTones);
	nxtDisplayString(0, "%d", time);
	wait1Msec(60000);
}
