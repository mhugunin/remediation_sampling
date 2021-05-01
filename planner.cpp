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
#include <unordered_set>
#include <memory>

using namespace std;
// #include "mex.h"

/* Input Arguments */
#define	ENVMAP_IN       prhs[0]
#define	OBSMAP_IN       prhs[1]
#define	EXPLROEDMAP_IN  prhs[2]
#define GOALMAP_IN      prhs[3]
#define ROBOT_IN        prhs[4]

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
vector <pair<int, int>> frontier; //@TODO: how do we update this?

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

bool willCollide(int x, int y, int* obstacleMap, int x_size, int y_size){
    return (int)obstacleMap[GETMAPINDEX(x, y, x_size, y_size)];
}

double sample(int x, int y, double* contaminationMap, int x_size, int y_size){
    return contaminationMap[GETMAPINDEX(x, y, x_size, y_size)];
}

class State {
public:
    State(int rpX, int rpY, shared_ptr<State> p, int id){
        robotposeX = rpX;
        robotposeY = rpY;
        this->parent = p;
        this->id = id;
    }
    float g = 0;
    float h = 0;
    float f = g + h;
    int robotposeX;
    int robotposeY;
    int id;
    shared_ptr<State> parent;
};

typedef shared_ptr<State> state_t;

class Compare{
    public:
      bool operator()(state_t n1, state_t n2) const {
            return n1->f > n2->f;
        }  
};

class Eq{
public:
    bool operator()(state_t n1, state_t n2) const{
        bool eq = (n1->robotposeX == n2->robotposeX && n1->robotposeY == n2->robotposeY);
        return eq;
    }
};

class Hash{
public:
    size_t operator()(const state_t &n) const {
        return std::hash<int>{}(n->id);
    }
};


vector<pair<int, int>> getPath(state_t curr, int* planLen){
    vector<pair<int, int>> path;
    state_t c = curr;
    pair<int, int> g = make_pair(curr->robotposeX, curr->robotposeY);
    path.push_back(g);
    int length = 0;
    while(c->parent != nullptr) {
        c = c->parent;
        pair<int, int> next = make_pair(c->robotposeX, c->robotposeY);
        path.push_back(next);
        length++;
    }
    *planLen = length;
    return path;
}

double euclideanDist(int x1, int y1, int x2, int y2){
    return sqrt(pow(x1-x2,2) + (pow(y1-y2,2)));
}

// multigoal backwards A*
static vector<pair<int, int>> planner(
            double* contaminationMap,
            int* obstacleMap,
            int* exploredMap,
            double* goalMap, //liklihood distribution
            int x_size, //size of obstacle/contamination Map
            int y_size,
            int robotposeX, //robot's current position = goal in backwards A*
            int robotposeY,
            int* plan_len
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
    priority_queue<state_t,vector<state_t>, Compare>open;
    unordered_set<state_t, Hash, Eq> closed;

    // find x,y of most likely contamination source from goalMap
    vector<int>::iterator maxElement;
    maxElement = max_element(goalMap.begin(), goalMap.end());
    int dist = distance(goalMap.begin(), maxElement);
    int sink_x = dist % x_size; //col
    int sink_y = dist / y_size; //row

    state_t sink = make_shared<State>(sink_x, sink_y, nullptr, 0); //sink node = most likely point on map to be contamination source

    int nextId = 1;
    vector<state_t> goals(frontier.size()); // vector of possible goals

    for(auto i: frontier){
        state_t n = make_shared<State>(i.first, i.second, sink, nextId);
        n->g =  euclideanDist(i.first, i.second, sink_x, sink_y);//euclidean distance to the most likely contamination source
        n->h = euclideanDist(robotposeX, robotposeY, i.first, i.second); // euclidean distance from node to robot's current position
        n->f = n->g + n->h;
        nextId ++;
    }

    open.push(sink);
    
    while(!open.empty()){
        // get current node
        state_t current_node = open.top();
        open.pop();
        // add to closed set
        auto iterator = closed.find(current_node);
        if(iterator != closed.end()){
            continue;
        }
        else {
            closed.insert(current_node);
        }
    
        // check if current node = start node = robot current position
        if (current_node->robotposeX == robotposeX && current_node->robotposeY == robotposeY) {
            vector<pair<int, int>> path = getPath(current_node, plan_len);
            return path;
        }
        if(current_node == sink){
            // generate successors for sink only
            for(state_t g: goals){
                open.push(g);
            }
        } else {
            // for every direction, generate children
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {   
                int newx = current_node->robotposeX + dX[dir];
                int newy = current_node->robotposeY + dY[dir];
               // check validity/within range
                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
                    if (!willCollide(newx, newy, obstacleMap, x_size, y_size)) { //if free
                        // create new nodes
                        state_t child_shared = make_shared<State>(newx, newy, current_node, nextId);
                        // check if children are in closed list
                        auto iterator = closed.find(child_shared);
                        if(iterator == closed.end()){ // children not in closed list
                             // else: set g,h,f
                            child_shared->g = (current_node -> g) + hypot((float)dX[dir],(float)dY[dir]);
                            //Euclidean distance
                            child_shared->h = euclideanDist(newx, newy, robotposeX, robotposeY);
                            //(max(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY) - min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)+min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)*sqrt(2.0));
                            child_shared->f = child_shared->g + child_shared->h;
                            // add to open list
                            open.push(child_shared);
                            nextId++;
                        } 
                    }
                }
            }
        }
    }
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
    if (nrhs != 5) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Five input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(ENVMAP_IN);
    int y_size = mxGetN(ENVMAP_IN);
    double* envmap = mxGetPr(ENVMAP_IN);
    int* obsmap = mxGetPr(OBSMAP_IN);
    int* exploredmap = mxGetPr(EXPLOREDMAP_IN);
    double* goalmap = mxGetPr(GOALMAP_IN);
    
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
        
    /* Create a matrix for the return action */ 

    int plan_len = 0;
            
    /* Do the actual planning in a subroutine */
    planner(envmap, obsmap, exploredmap, goalmap, x_size, y_size, robotposeX, robotposeY, &plan_len);

    // planner needs to return / set a vector or something for actual plan

    // set the output to returned plan
    plhs[0] = mxCreateNumericMatrix( (mwSize)2, (mwSize)plan_len, mxINT32_CLASS, mxREAL); 
    int* action_ptr = (int*)  mxGetPr(plhs[0]);

    // loop through and assign to action_ptr for each location in plan
    return;
    
}