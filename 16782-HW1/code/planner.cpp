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
  std::priority_queue<State*, std::vector<State*>, CompareStates> open_list;

  // Create a closed list to keep track of explored states
  std::vector<std::vector<bool>> closed_list(x_size, std::vector<bool>(y_size, false));

  // Initialize the start state just once
  static State* start = new State{ 0, 0, 0, 0, nullptr };  // TODO: this should only be done once?
  if (flag == 0)
  {
    start->x = robotposeX;
    start->y = robotposeY;
    open_list.push(start);
    flag = 1;
  }

  // Initialize the goal state
  int goalposeX = target_traj[target_steps - 1];
  int goalposeY = target_traj[target_steps - 1 + target_steps];
  State* goal = new State{ goalposeX, goalposeY, 0, 0, nullptr };

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
    std::cout << "newx: " << newx << " newy: " << newy << std::endl;
    action_ptr[0] = newx;
    action_ptr[1] = newy;
    std::cout << "action: " << action_ptr[0] << " " << action_ptr[1] << std::endl;
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
    State* current = open_list.top();
    open_list.pop();

    // Check if the current state is the goal state
    if (current->x == goal->x && current->y == goal->y)
    {
      // Backtrack to extract the optimal path
      State* temp = current;
      while (temp->parent != nullptr)
      {
        // store the whole path in a desired robot trajectory
        robot_traj.push_back(std::make_pair(temp->x, temp->y));
        temp = temp->parent;
      }
      is_planning_done = true;
      get_action();
      return;
    }

    // Add the current state to the closed list
    closed_list[current->x][current->y] = true;

    // Expand the current state
    for (int dir = 0; dir < NUMOFDIRS; dir++)
    {
      int newx = current->x + dX[dir];
      int newy = current->y + dY[dir];

      // Check if the new state is within the map
      if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
      {
        // Check if the new state is free
        if ((map[GETMAPINDEX(newx, newy, x_size, y_size)] >= 0) &&
            (map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh))
        {
          // Check if the new state is in the closed list
          if (!closed_list[newx][newy])
          {
            // Make a new state for the new state
            State* new_state = new State{ newx, newy, current->g + 1, 0, current };

            // Compute the heuristic for the new state
            // TODO: use diagonal distance instead of Euclidean distance
            new_state->h = (int)sqrt((pow(new_state->x - goal->x, 2) + pow(new_state->y - goal->y, 2)));

            // Check if the new state is in the open list
            // TODO: make this efficient
            bool in_open_list = false;
            for (int i = 0; i < open_list.size(); i++)
            {
              State* temp = open_list.top();
              open_list.pop();
              if (temp->x == new_state->x && temp->y == new_state->y)
              {
                in_open_list = true;
                // If the new state is in the open list, update the cost and parent if necessary
                if (new_state->g < temp->g)
                {
                  temp->g = new_state->g;
                  temp->parent = new_state->parent;
                }
              }
              open_list.push(temp);
            }

            // If the new state is not in the open list, add the new state to the open list
            if (!in_open_list)
            {
              open_list.push(new_state);
            }
          }
        }
      }
    }
  }
  // return;
}