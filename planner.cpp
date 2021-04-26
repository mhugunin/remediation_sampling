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
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

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

static void planner(
		   double*	map,
		   int x_size,
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
    
    //printf("robot: %d %d; ", robotposeX, robotposeY);
    //printf("goal: %d %d;", goalposeX, goalposeY);
    
	//The planner:
    clock_t start = clock(); //Start a timer
    double duration = 0.200 * 0.8; //Duration (200 ms) * small factor of safety for when to accept solution and end planner
	int start_h = heuristic(robotposeX-goalposeX, robotposeY-goalposeY); //starting heuristic 'distance'
    int epsilon = MAX(2, (int) (0.3*start_h)); //Starting epsilon chosen to confidently complete min 1 planning cycle within duration

    double elapsed = double(clock()-start)/CLOCKS_PER_SEC;
    
    bool direction_found = 0; //whether A direction for the next move has been found
    
    //Instantiate priority queue for open list
    std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, Compare > open;
    
    //Initialize vectors for closed list properties
    std::vector<int> closed;        //id
    std::vector<int> closed_g;      //cost g
    std::vector<int> closed_backp;  //index of closed list node which -> this closed list node
        
    while(!direction_found || (epsilon > 1 && elapsed < duration)){
        epsilon = (int) (epsilon/2); //each round of planning will follow with eps/2
    
        //Initialize first element in open list (robot node)
        int g0 = 0;
        int h0 = heuristic(robotposeX-goalposeX, robotposeY-goalposeY);
        open.push({epsilon*h0+g0, h0, g0, xy2count(robotposeX, robotposeY, y_size), -1});
        
        //Store closed list inconsistent nodes to be reused
        std::queue<std::vector<int>> incons;
    
        //End condition of A*
        bool goal_expanded = 0;

        while(!goal_expanded && !open.empty() && elapsed < duration){
            //Read clock to ensure we don't run out of time mid-plan
            elapsed = double(clock()-start)/CLOCKS_PER_SEC;

            //Expand top of open priority queue
            std::vector<int> e_cell = open.top();
            open.pop();

            closed.push_back(e_cell[3]); //add node, g value to closed lists
            closed_g.push_back(e_cell[2]);
            closed_backp.push_back(e_cell[4]);

            //Convert cell ID to map coordinates
            int cell_x = xcord(e_cell[3], y_size);
            int cell_y = ycord(e_cell[3], y_size);

            //Check if the cell being expanded is the goal cell
            if(cell_x == goalposeX && cell_y == goalposeY){
                goal_expanded = 1;
                //printf("goal has been expanded");
            }
            else{
                //Run through all 8-way neightbors
                for(int dir = 0; dir < NUMOFDIRS; dir++)
                {
                    int newx = cell_x + dX[dir]; //Coordinates of neighbor node
                    int newy = cell_y + dY[dir];

                    bool inbounds = (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size);
                    bool notclosed = (std::count(closed.begin(), closed.end(), xy2count(newx, newy, y_size))==0);
                    if(inbounds){//Check the neighbor is in map bounds and not already expanded
                        bool notoccupied = ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] == 0);
                        if (notoccupied){ //Check the neighbor is in map free space
                            int new_h = heuristic(newx-goalposeX, newy-goalposeY); //Heuristic for the new node
                            int new_g = e_cell[2] + 1; //Cost is 1 for any move
                            if(notclosed){
                                //Push the new node to the open list
                                open.push({epsilon*new_h+new_g, new_h, new_g, xy2count(newx, newy, y_size), (int)(closed_backp.size()-1)});
                            }
                            else{
                                incons.push({epsilon*new_h+new_g, new_h, new_g, xy2count(newx, newy, y_size), (int)(closed_backp.size()-1)});
                            }
                        }
                    }
                }
            }
        }

        //backtrace through expanded nodes to find the first move
        if(goal_expanded){ //Check the main planning loop didn't terminate by timeout or by an empty open list
            int closed_ptr = closed_backp.size()-1;
            while(closed_ptr != -1){ //Step back through nodes pointing to goal, making sure list end is not reached
                closed_ptr = closed_backp[closed_ptr];
                if(closed_g[closed_ptr] == 1){ //When the first node after the robot is reached
                    *p_actionX = xcord(closed[closed_ptr], y_size) - robotposeX; //Next action is defined by motion to this first node
                    *p_actionY = ycord(closed[closed_ptr], y_size) - robotposeY;
                    direction_found = 1; //Code may now terminate anytime if 200ms runtime nears
                    break;
                }
            }
            //Recycle open, incons:
            if(epsilon > 1){
                int n_eps = (int) (epsilon/2);
                std::priority_queue<std::vector<int>, std::vector<std::vector<int>>, Compare > nextopen;
                while (!open.empty()){
                    std::vector<int> e_cell = open.top();
                    open.pop();
                    nextopen.push({n_eps*e_cell[1]+e_cell[2], e_cell[1], e_cell[2], e_cell[3], e_cell[4]});
                }
                while (!incons.empty()){
                    std::vector<int> e_cell = incons.front();
                    incons.pop();
                    nextopen.push({n_eps*e_cell[1]+e_cell[2], e_cell[1], e_cell[2], e_cell[3], e_cell[4]});
                }
                open = nextopen;
            }
        }
    }
    //p_action values are now final
    
    //Optionally print planning info to terminal
    //printf("epsilon terminal: %d", epsilon);
    //printf("action: %d %d; \n", *p_actionX, *p_actionY);
    //printf("Time measured: %.4f seconds.\n", elapsed);
    
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
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
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
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]); 
    return;
    
}