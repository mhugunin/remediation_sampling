/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <queue> 
#include <vector>
#include <algorithm>
#include <time.h>
#include "mex.h"

/* Input Arguments */
#define	ENVMAP_IN   prhs[0]
#define	OBSMAP_IN   prhs[1]
#define ROBOT_IN    prhs[2]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

int temp = 0;

//3 functions to go between a unique node id <-> x/y on map
int xcord(int counter, int y_size){
    return counter/y_size;
}
int ycord(int counter, int y_size){
    return counter%y_size;
}
int xy2count(int x, int y, int y_size){
    return x*y_size + y;
}

//Define heuristic by euclidean dist/sqrt(2) (accounts for diagonal weight 1)
int heuristic(int dx, int dy){
    return (int)(sqrt(((dx)*(dx) + (dy)*(dy)))/1.414);
}

struct Compare{ //Operator for priority of queue by minimum f=h+g
    bool operator()(std::vector<int> a, std::vector<int> b){
        return (a[0])>(b[0]);
    }
};

bool willCollide(int x, int y, int* obstacleMap, int x_size, int y_size){
    return (int)obstacleMap[GETMAPINDEX(x, y, x_size, y_size)];
}

double sample(int x, int y, double* contaminationMap, int x_size, int y_size){
    return contaminationMap[GETMAPINDEX(x, y, x_size, y_size)];
}

class State {
public:
    State(int rpX, int rpY, double* probability, double* exploredSoFar, shared_ptr<Node> p){
        robotposeX = rpX;
        robotposeY = rpY;
        this->probability = probability;
        this->exploredSoFar = exploredSoFar;
        this->parent = p;
    }
    double* probability;
    double* exploredSoFar;
    
    float g = 0;
    float h = 0;
    float f = g + h;
    int robotposeX;
    int robotposeY;
    shared_ptr<Node> parent;
};
class Compare{
    public:
      bool operator()(shared_ptr<State> n1,shared_ptr<State> n2){
            return n1->f > n2->f;
        }  
};

typedef shared_ptr<State> state_t;

pair<int, int> getNextStep(node_t curr){
    node_t c = curr;
    int xd=0;
    while(c->parent->parent != nullptr) {
        c = c->parent;
        xd++;
    }
    int nextX = c->robotposeX;
    int nextY = c->robotposeY;
    pair<int, int> n = make_pair(nextX, nextY);
    return n;
}


static void planner(
		   double* contaminationMap,
           int* obstacleMap,
		   int x_size, //size of obstacle/contamination Map
 		   int y_size,
           int robotposeX,
            int robotposeY,
            int goalposeX,
            int goalposeY,
            char *p_actionX,
            char *p_actionY
		   )
{
       //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};    
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    printf("call=%d\n", temp);
    temp = temp+1;
    
//     printf("robot: %d %d; ", robotposeX, robotposeY);
//     printf("goal: %d %d;", goalposeX, goalposeY);

    //A* planner
    priority_queue<node_t,vector<node_t>, Compare>open;
    vector<vector<char>> closed(y_size+1, vector<char> (x_size+1, 0));
    
    node_t start_shared = make_shared<Node>(robotposeX, robotposeY, nullptr);
    open.push(start_shared);
    
    while(!open.empty()){
        // get current node
        node_t current_node = open.top();
        open.pop();
        // add to closed list
        if(closed[current_node->robotposeY][current_node->robotposeX] == 1){
            continue;
        }
        else{
            closed[current_node->robotposeY][current_node->robotposeX] = 1;
        }
        // check if = target
        if (current_node->robotposeX == goalposeX && current_node->robotposeY == goalposeY) {
            auto res = getNextStep(current_node);
            *p_actionX = res.first - start_shared->robotposeX; 
            *p_actionY = res.second - start_shared->robotposeY;
            return;
        }
        // for every direction, generate children
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {   
            int newx = current_node->robotposeX + dX[dir];
            int newy = current_node->robotposeY + dY[dir];
           // check validity/within range
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
                if (!willCollide(newx, newy, obstacleMap, x_size, y_size)){ //if free
                    // create new nodes
                    node_t child_shared = make_shared<Node>(newx, newy, current_node);
                    // check if children are in closed list
                    if(closed[newy][newx] != 1){ // children not in closed list
                         // else: set g,h,f
                        child_shared->g = (current_node -> g) + hypot((float)dX[dir],(float)dY[dir]);
                        //Euclidean distance
                        child_shared->h = sqrt(pow(child_shared->robotposeX - goalposeX,2) + (pow(child_shared->robotposeY - goalposeY,2)));
                        //(max(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY) - min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)+min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)*sqrt(2.0));
                        child_shared->f = child_shared->g + child_shared->h;
                        // add to open list
                        open.push(child_shared);
                    } 
                }
            }
        }
    }
   
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector <x,y> for the robot pose
//3rd is a row vector <x,y> for the target pose
//plhs should contain output parameters (1): 
//1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    }// } else if (nlhs != 1) {
	//     mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
    //             "One output argument required."); 
    // } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(ENVMAP_IN);
    int y_size = mxGetN(ENVMAP_IN);
    double* envmap = mxGetPr(ENVMAP_IN);
    double* obsmap = mxGetPr(OBSMAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
        
    // /* Create a matrix for the return action */ 
    // ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    // char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    planner(envmap, obsmap, x_size, y_size, robotposeX, robotposeY); 
    return;
    
}