/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <vector>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// Define a struct for representing a state in the search space
struct State
{
  int x, y;
  int g;          // Cost from the start node to this node
  int h;          // Heuristic (e.g., Euclidean distance to the target)
  State* parent;  // Pointer to the parent state
};

// Define a comparison function for the priority queue
struct CompareStates
{
  bool operator()(const State* a, const State* b) const
  {
    return (a->g + a->h) > (b->g + b->h);
  }
};

void planner(int* map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps,
             int* target_traj, int targetposeX, int targetposeY, int curr_time, int* action_ptr)
{
  // 8-connected grid
  int dX[NUMOFDIRS] = { -1, -1, -1, 0, 0, 1, 1, 1 };
  int dY[NUMOFDIRS] = { -1, 0, 1, -1, 1, -1, 0, 1 };

  // implement A* here to catch target with minimum steps and minimum cost
  // Create an open list (priority queue) for states to explore
  std::priority_queue<State*, std::vector<State*>, CompareStates> open_list;

  // Create a closed list to keep track of explored states
  std::vector<std::vector<bool>> closed_list(x_size, std::vector<bool>(y_size, false));

  // Initialize the start state
  State* start = new State{ robotposeX, robotposeY, 0, 0, nullptr };
  open_list.push(start);

  // Initialize the goal state
  int goalposeX = target_traj[target_steps - 1];
  int goalposeY = target_traj[target_steps - 1 + target_steps];

  

  return;
}