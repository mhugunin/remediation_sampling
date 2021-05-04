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
#define GOAL_IN      prhs[3]
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
    state_t c = curr->parent; // the frontier node
    // pair<int, int> g = make_pair(c->robotposeX, c->robotposeY);
    // path.push_back(g);

    int length = 0;
    while(c->parent != nullptr) {
        ////printf("%d\n", __LINE__);
        pair<int, int> next = make_pair(c->robotposeX, c->robotposeY);
        path.push_back(next);
        length++;
        c = c->parent;
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
            int x_size, //size of obstacle/contamination Map
            int y_size,
            int robotposeX, //robot's current position = goal in backwards A*
            int robotposeY,
            int goalposeX,
            int goalposeY,
            int* plan_len
		   )
{
    //8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};    
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

        // Get frontier from goalMap
    if(firstCall){ // first time calling planner
        //////printf("first Call: %d\n", __LINE__);
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
    //////printf("call=%d\n", temp);
    temp = temp+1;
    
//     ////printf("robot: %d %d; ", robotposeX, robotposeY);
//     ////printf("goal: %d %d;", goalposeX, goalposeY);

    //A* planner
    priority_queue<state_t,vector<state_t>, Compare>open;
    unordered_set<state_t, Hash, Eq> closed;

    // find x,y of most likely contamination source from goalMap
    double mostLikely = std::numeric_limits<double>::lowest();
    int sink_x = goalposeX;
    int sink_y = goalposeY;
    ////printf("Robot: (%d, %d)\n", robotposeX, robotposeY);
    ////printf("goal: (%d, %d)\n", sink_x, sink_y);
    // for(int i = 0; i < y_size; ++i){
    //     for(int j = 0; j < x_size; ++j){
    //         if(goalMap[GETMAPINDEX(j+1,i+1, x_size, y_size)] > mostLikely){
    //             mostLikely = goalMap[GETMAPINDEX(j+1,i+1, x_size, y_size)];
    //             sink_x = j + 1;
    //             sink_y = i + 1;
    //         }
    //     }
    // }

     //sink node = most likely point on map to be contamination source

    int nextId = 1;

    // vector<state_t> goals; // vector of possible goals
    //printf("Frontier size: %d, line: %d\n", frontier.size(), __LINE__);

    bool robot_at_sink = (sink_x == robotposeX && sink_y == robotposeY);

    // add robot pos to open list
    state_t start = make_shared<State>(robotposeX, robotposeY, nullptr, 0);
    start->g = 0;
    if (robot_at_sink) {
        start->h = 0;
    }
    else {
        start->h = euclideanDist(robotposeX, robotposeY, sink_x, sink_y);
    }
    start->f = start->g + start->h;
    open.push(start);


    // open.push(sink);

    ////printf("pre while loop: %d\n", __LINE__);
    
    while(!open.empty()){
        // get current node
        state_t current_node = open.top();
        open.pop();
        // //printf("Current Node ID: %d, line: %d\n", current_node->id, __LINE__);
        //printf("Current Node loc: (%d, %d)\n", current_node->robotposeX, current_node->robotposeY);
        // add to closed set
        auto iterator = closed.find(current_node);
        if(iterator != closed.end()){
            continue;
        }
        else {
            closed.insert(current_node);
        }

        // check if current node = sink
        if (!robot_at_sink && current_node->robotposeX == sink_x && current_node->robotposeY == sink_y) {

            //printf("Found sink, about to get path: %d\n", __LINE__);
            vector<pair<int, int>> path = getPath(current_node, plan_len);

            if (exploredMap[GETMAPINDEX(sink_x, sink_y, x_size, y_size)] == 1 || frontier.find(make_pair(sink_x, sink_y)) != frontier.end()) {
                // add sink to path
                //printf("%d\n", __LINE__);
                // check if current_node is within range of parent
                state_t sink_parent = current_node->parent;
                if (euclideanDist(current_node->robotposeX, current_node->robotposeY, sink_parent->robotposeX, sink_parent->robotposeY) < 1.5) {
                    path.insert(path.begin(), make_pair(sink_x, sink_y));
                    *plan_len = *plan_len + 1;
                }
            }
            // expand the frontier from last node
            //printf("%d\n", __LINE__);
            //printf("%d\n", path.size());
            if (current_node->parent == nullptr) {
                //printf("REALLY BAD\n");
            }
            auto chosenGoal = path[0];
            //printf("%d\n", __LINE__);
            // remove the frontier node
            if (exploredMap[GETMAPINDEX(sink_x, sink_y, x_size, y_size)] != 1) {
                //printf("%d\n", __LINE__);
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
                            // //printf("Inserted into frontier!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            // //printf("location: (%d, %d)\n", prospective.first, prospective.second);
                        }
                    }
                }
            }

            return path;
        }
        if(frontier.find(make_pair(current_node->robotposeX, current_node->robotposeY)) != frontier.end()){
            // current node is on the frontier
            // //printf("Goals size: %d, line: %d\n", goals.size(), __LINE__);

            if (robot_at_sink) {
                // return a path
                //printf("Found frontier, about to get path: %d\n", __LINE__);
                vector<pair<int, int>> path = getPath(current_node, plan_len);
                // append frontier node
                path.insert(path.begin(), make_pair(current_node->robotposeX, current_node->robotposeY));
                *plan_len = *plan_len + 1;

                auto chosenGoal = path[0];
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
                            // //printf("Inserted into frontier!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            // //printf("location: (%d, %d)\n", prospective.first, prospective.second);
                        }
                    }
                }
                return path;
            }

            //printf("expanding frontier node!\n");
            state_t sink = make_shared<State>(sink_x, sink_y, current_node, nextId);
            sink->g = current_node->g + euclideanDist(current_node->robotposeX, current_node->robotposeY, sink_x, sink_y);//euclidean distance to the most likely contamination source
            sink->h = 0; // euclidean distance from node to robot's current position
            sink->f = sink->g;
            open.push(sink);
            nextId++;
        } else {

            // for every direction, generate children
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {   
                int newx = current_node->robotposeX + dX[dir];
                int newy = current_node->robotposeY + dY[dir];
               // check validity/within range
                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size){
                    if ((exploredMap[GETMAPINDEX(newx, newy, x_size, y_size)] == 1) || (frontier.find(make_pair(newx, newy)) != frontier.end())) { //if in explored region or frontier
                        // create new nodes
                        state_t child_shared = make_shared<State>(newx, newy, current_node, nextId);
                        // check if children are in closed list
                        auto iterator = closed.find(child_shared);
                        if(iterator == closed.end()){ // children not in closed list
                             // else: set g,h,f
                            child_shared->g = (current_node -> g) + hypot((float)dX[dir],(float)dY[dir]);
                            //Euclidean distance
                            if (robot_at_sink) {
                                child_shared->h = 0;
                            }
                            else {
                                child_shared->h = euclideanDist(newx, newy, sink_x, sink_y);
                            }
                            
                            //(max(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY) - min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)+min(child_shared->robotposeX - goalposeX, child_shared->robotposeY - goalposeY)*sqrt(2.0));
                            child_shared->f = child_shared->g + child_shared->h;
                            // add to open list
                            open.push(child_shared);
                            nextId++;
                            //printf("about to add child: (%d, %d)\n", newx, newy);
                        } 
                    }
                }
            }
        }
    }

    ////printf("Broke out of while loop: %d\n", __LINE__);
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

    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];

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

    // //printf("About to call planner: %d\n", __LINE__);
            
    /* Do the actual planning in a subroutine */
    vector<pair<int, int>> path = planner(envmap, obsmap, exploredmap, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &plan_len);

    // planner needs to return / set a vector or something for actual plan
    //printf("Path size: %d\n", path.size());
    //printf("Plan len: %d\n", plan_len);
    
    // set the output to returned plan
    plhs[0] = mxCreateNumericMatrix( (mwSize)plan_len, (mwSize)2, mxINT32_CLASS, mxREAL);
    int* action_ptr = (int*)  mxGetPr(plhs[0]);

    // loop through and assign to action_ptr for each location in plan

    for(int i = 0; i < path.size(); ++i){
        ////printf("Path node loc: (%d, %d)\n", path[plan_len - 1 - i].first, path[plan_len - 1 - i].second);
        
        action_ptr[GETMAPINDEX(i+1,1, plan_len, 2)] = path[plan_len - 1 - i].first;
        action_ptr[GETMAPINDEX(i+1,2, plan_len, 2)] = path[plan_len - 1 - i].second;
    }
    return;
    
}