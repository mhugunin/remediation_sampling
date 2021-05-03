/*=================================================================
 *
 * planner.c
    X IS NUMBER OF COLUMNS, Y IS NUMBER OF ROWS!
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
#include "mex.h"

/* Input Arguments */
#define	ENVMAP_IN       prhs[0]
#define	OBSMAP_IN       prhs[1]
#define	EXPLOREDMAP_IN  prhs[2]
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
int firstCall = 1;

class frontierHash{
public:
    size_t operator()(const pair<int, int> &n) const {
        return (size_t)n.first << 32 & (size_t)n.second;
    }
};

class frontierEq{
public:
    bool operator()(pair<int, int> n1, pair<int, int> n2) const{
        bool eq = (n1.first == n2.first && n1.second == n2.second);
        return eq;
    }
};

unordered_set <pair<int, int>, frontierHash, frontierEq> frontier;

bool willCollide(int x, int y, bool* obstacleMap, int x_size, int y_size){
    return (bool)(int)obstacleMap[GETMAPINDEX(x, y, x_size, y_size)];
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
    while(c->parent->parent != nullptr) {
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
            bool* obstacleMap,
            bool* exploredMap,
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

        // Get frontier from goalMap
    if(firstCall){ // first time calling planner
        printf("first Call: %d\n", __LINE__);
        // add surronding nodes of the current robot positions
        for (int dir = 0; dir < NUMOFDIRS; dir++)
            {   
                int newx = robotposeX + dX[dir];
                int newy = robotposeY + dY[dir];
                //collision check and then add to frontier vector
                if(!willCollide(newx, newy, obstacleMap, x_size, y_size)){
                    frontier.insert(make_pair(newx, newy));
                }
            }
    }
    firstCall = 0;
    printf("call=%d\n", temp);
    temp = temp+1;
    
//     printf("robot: %d %d; ", robotposeX, robotposeY);
//     printf("goal: %d %d;", goalposeX, goalposeY);

    //A* planner
    priority_queue<state_t,vector<state_t>, Compare>open;
    unordered_set<state_t, Hash, Eq> closed;

    // find x,y of most likely contamination source from goalMap
    double mostLikely = std::numeric_limits<double>::lowest();
    int sink_x = 0;
    int sink_y = 0;
    for(int i = 0; i < y_size; ++i){
        for(int j = 0; j < x_size; ++j){
            if(goalMap[GETMAPINDEX(j+1,i+1, x_size, y_size)] > mostLikely){
                mostLikely = goalMap[GETMAPINDEX(j+1,i+1, x_size, y_size)];
                sink_x = j + 1;
                sink_y = i + 1;
            }
        }
    }

    state_t sink = make_shared<State>(sink_x, sink_y, nullptr, 0); //sink node = most likely point on map to be contamination source

    int nextId = 1;
    vector<state_t> goals; // vector of possible goals
    printf("Frontier size: %d, line: %d\n", frontier.size(), __LINE__);

    if (exploredMap[GETMAPINDEX(sink_x, sink_y, x_size, y_size)] == 1 || frontier.find(sink) != frontier.end()) {
        // add sink to goals vector
        state_t sink_copy = make_shared<State>(sink_x, sink_y, sink, -1);
        goals.push_back(sink_copy);
    }
    else {
        for(auto i: frontier){
            state_t n = make_shared<State>(i.first, i.second, sink, nextId);
            n->g =  euclideanDist(i.first, i.second, sink_x, sink_y);//euclidean distance to the most likely contamination source
            n->h = euclideanDist(robotposeX, robotposeY, i.first, i.second); // euclidean distance from node to robot's current position
            n->f = n->g + n->h;
            nextId ++;
            goals.push_back(n);
        }
    }

    printf("Goals size: %d, line: %d\n", goals.size(), __LINE__);

    open.push(sink);

    printf("pre while loop: %d\n", __LINE__);
    
    while(!open.empty()){
        // get current node
        state_t current_node = open.top();
        open.pop();
        // printf("Current Node ID: %d, line: %d\n", current_node->id, __LINE__);
        // printf("Current Node loc: (%d, %d)\n", current_node->robotposeX, current_node->robotposeY);
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

            // printf("Found robot, about to get path: %d\n", __LINE__);
            vector<pair<int, int>> path = getPath(current_node, plan_len);
            // expand the frontier from last node
            auto chosenGoal = path[path.size()-1];
            // remove the frontier node
            frontier.erase(chosenGoal);
            exploredMap[GETMAPINDEX(chosenGoal.first, chosenGoal.second, x_size, y_size)] = true;
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {   
                int newx = chosenGoal.first + dX[dir];
                int newy = chosenGoal.second + dY[dir];

                pair<int, int> prospective = make_pair(newx, newy);
                //collision check and then add to frontier vector
                if(!willCollide(newx, newy, obstacleMap, x_size, y_size)){
                    if(!exploredMap[GETMAPINDEX(newx, newy, x_size, y_size)] && frontier.find(prospective) == frontier.end()){ // not already in explored map and not already in frontier
                        frontier.insert(prospective);
                        // printf("Inserted into frontier!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                        // printf("location: (%d, %d)\n", prospective.first, prospective.second);
                    }
                }
            }

            return path;
        }
        if(current_node == sink){
            // generate successors for sink only
            // printf("Goals size: %d, line: %d\n", goals.size(), __LINE__);
            for(state_t g: goals){
                // printf("goals node ID: %d, line: %d\n", g->id, __LINE__);
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

    printf("Broke out of while loop: %d\n", __LINE__);
    vector<pair<int, int>> default_vec;
    default_vec.push_back(make_pair(-1, -1));
    return default_vec;
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
    bool* obsmap = mxGetLogicals(OBSMAP_IN);
    bool* exploredmap = mxGetLogicals(EXPLOREDMAP_IN);
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

    // printf("About to call planner: %d\n", __LINE__);
            
    /* Do the actual planning in a subroutine */
    vector<pair<int, int>> path = planner(envmap, obsmap, exploredmap, goalmap, x_size, y_size, robotposeX, robotposeY, &plan_len);

    // planner needs to return / set a vector or something for actual plan
    // printf("Path size: %d\n", path.size());
    printf("Plan len: %d\n", plan_len);
    
    // set the output to returned plan
    plhs[0] = mxCreateNumericMatrix( (mwSize)plan_len, (mwSize)2, mxINT32_CLASS, mxREAL);
    int* action_ptr = (int*)  mxGetPr(plhs[0]);

    // loop through and assign to action_ptr for each location in plan
    for(int i = 1; i < path.size(); ++i){
        printf("Path node loc: (%d, %d)\n", path[i].first, path[i].second);
        
        action_ptr[GETMAPINDEX(i,1, plan_len, 2)] = path[i].first;
        action_ptr[GETMAPINDEX(i,2, plan_len, 2)] = path[i].second;
    }
    return;
    
}