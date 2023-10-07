/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <vector>
#include <queue>
#include <iostream>
#include <unordered_set>

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
  bool has_been_visited = false;
};

// Define a comparison function for the priority queue
struct CompareStates
{
  bool operator()(const State a, const State b) const
  {
    return (a.g + a.h) > (b.g + b.h);
  }
};

static int flag = 0;

// 8-connected grid
int dX[NUMOFDIRS] = { -1, -1, -1, 0, 0, 1, 1, 1 };
int dY[NUMOFDIRS] = { -1, 0, 1, -1, 1, -1, 0, 1 };

// Desired robot trajectory
std::vector<std::pair<int, int>> robot_traj;
static bool is_planning_done = false;

void planner(int* map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps,
             int* target_traj, int targetposeX, int targetposeY, int curr_time, int* action_ptr)
{
  // Create an open list (priority queue) for states to explore
  static std::priority_queue<State, std::vector<State>, CompareStates> open_list;

  // Create a closed list to keep track of explored states
  static std::vector<std::vector<bool>> closed_list(x_size, std::vector<bool>(y_size, false));

  // static std::vector<std::vector<bool>> visited_list(x_size, std::vector<bool>(y_size, false));
  static std::vector<std::vector<State>> open_list_state(x_size, std::vector<State>(y_size));

  // Initialize the start state just once
  static State start = State{ 0, 0, 0, 0, nullptr };
  if (flag == 0)
  {
    start.x = robotposeX;
    start.y = robotposeY;
    start.has_been_visited = true;
    open_list.push(start);
    open_list_state[robotposeX - 1][robotposeY - 1] = start;
    flag = 1;
  }

  // Initialize the goal state
  int goalposeX = target_traj[target_steps - 1];
  int goalposeY = target_traj[target_steps - 1 + target_steps];
  State goal = State{ goalposeX, goalposeY, 0, 0, nullptr };

  auto get_action = [&] {
    if (robot_traj.empty())
    {
      int newx = robotposeX;
      int newy = robotposeY;
      action_ptr[0] = newx;
      action_ptr[1] = newy;
      return;
    }
    int newx = robot_traj.back().first;
    int newy = robot_traj.back().second;
    robot_traj.pop_back();
    action_ptr[0] = newx;
    action_ptr[1] = newy;
    return;
  };

  action_ptr[0] = robotposeX;
  action_ptr[1] = robotposeY;

  // Lambda function to backtrack the optimal path
  auto backtrack_path = [&](State goal) {
    // Backtrack to extract the optimal path
    State temp = goal;
    // std::cout << "backtrack_path" << std::endl;
    while (temp.parent != nullptr)
    {
      // store the whole path in a desired robot trajectory
      robot_traj.push_back(std::make_pair(temp.x, temp.y));
      temp = *temp.parent;
    }
    return;
  };

  if (is_planning_done)
  {
    get_action();
    return;
  }

  while (!open_list.empty())
  {
    // Pop the state with the lowest cost from the open list
    State current = open_list.top();
    open_list.pop();
    open_list_state[current.x - 1][current.y - 1] = current;
    // Add the current state to the closed list
    closed_list[current.x - 1][current.y - 1] = true;
    // Check if the current state is the goal state
    if (current.x == goal.x && current.y == goal.y)
    {
      is_planning_done = true;
      backtrack_path(current);
      get_action();
      break;
    }

    // Expand the current state
    for (int dir = 0; dir < NUMOFDIRS; dir++)
    {
      int newx = current.x + dX[dir];
      int newy = current.y + dY[dir];
      // Check if the new state is within the map
      if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
      {
        // Check if the new state is free
        int cell_cost = (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
        if ((cell_cost >= 0) && (cell_cost < collision_thresh))
        {
          // Check if the new state is in the closed list
          if (!closed_list[newx - 1][newy - 1])
          {
            // Make a new state for the new state
            State new_state = State{ newx, newy, current.g + cell_cost, 0, nullptr };
            new_state.parent = &open_list_state[current.x - 1][current.y - 1];

            // Compute the heuristic for the new state. Diagonal distance is used here.
            new_state.h = MAX(abs(new_state.x - goal.x), abs(new_state.y - goal.y));
            // new_state.h = sqrt(pow((new_state.x - goal.x),2) + pow((new_state.y - goal.y),2));

            // Check if the new state is in the open list
            bool in_open_list = open_list_state[new_state.x - 1][new_state.y - 1].has_been_visited;
            if (!in_open_list)
            {
              // Add the new state to the open list and the open set
              new_state.has_been_visited = true;
              open_list.push(new_state);
              open_list_state[new_state.x - 1][new_state.y - 1] = new_state;
            }
          }
        }
      }
    }
  }
}