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

  // Define a custom equality operator for the State class
  bool operator==(const State& s) const
  {
    return (x == s.x && y == s.y);
  }
};

// Define a comparison function for the priority queue
struct CompareStates
{
  bool operator()(const State* a, const State* b) const
  {
    return (a->g + a->h) > (b->g + b->h);
  }
};

// Define a custom hash function for State class
// template<> struct std::hash<State>
// {
//   size_t operator()(const State s) const noexcept
//   {
//     return std::hash<int>()(s.x) ^ std::hash<int>()(s.y);
//     // return s->x * 5000 + s->y;
//   }
// };

struct StateHash
{
  size_t operator()(const State s) const
  {
    return std::hash<int>()(s.x) ^ std::hash<int>()(s.y);
    // return s->x * 5000 + s->y;
  }
};

// Create an unordered set to store the states in the open list
std::unordered_set<State, StateHash> open_set;
// std::unordered_set<State, std::hash<State>> open_set;

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
  static State* start = new State{ 0, 0, 0, 0, nullptr };
  if (flag == 0)
  {
    start->x = robotposeX;
    start->y = robotposeY;
    open_list.push(start);
    open_set.insert(*start);
    flag = 1;
  }

  // Initialize the goal state
  int goalposeX = target_traj[target_steps - 1];
  int goalposeY = target_traj[target_steps - 1 + target_steps];
  State* goal = new State{ goalposeX, goalposeY, 0, 0, nullptr };

  auto get_action = [&] {
    if (robot_traj.empty())
    {
      std::cout << "robot_traj empty" << std::endl;
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

  // Lambda function to backtrack the optimal path
  auto backtrack_path = [&](State* goal) {
    // Backtrack to extract the optimal path
    State* temp = goal;
    while (temp->parent != nullptr)
    {
      // store the whole path in a desired robot trajectory
      robot_traj.push_back(std::make_pair(temp->x, temp->y));
      temp = temp->parent;
    }
    std::cout << "robot_traj size: " << robot_traj.size() << std::endl;
    return;
  };

  if (is_planning_done)
  {
    std::cout << "getting action" << std::endl;
    get_action();
    return;
  }

  while (!open_list.empty())
  {
    // Pop the state with the lowest cost from the open list
    State* current = open_list.top();
    open_list.pop();
    std::cout << "current: " << current->x << " " << current->y << std::endl;
    // Add the current state to the closed list
    closed_list[current->x][current->y] = true;  // FIXME: Indexing is incorrect here?
    // Check if the current state is the goal state
    if (current->x == goal->x && current->y == goal->y)
    {
      std::cout << "Found plan!" << std::endl;
      is_planning_done = true;
      backtrack_path(current);
      get_action();
      break;
    }

    // Expand the current state
    for (int dir = 0; dir < NUMOFDIRS; dir++)
    {
      int newx = current->x + dX[dir];
      int newy = current->y + dY[dir];

      // Check if the new state is within the map
      if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
      {
        // Check if the new state is free
        int cell_cost = (int)map[GETMAPINDEX(newx, newy, x_size, y_size)];
        if ((cell_cost >= 0) && (cell_cost < collision_thresh))
        {
          // Check if the new state is in the closed list
          if (!closed_list[newx][newy])
          {
            // Make a new state for the new state
            State new_state = State{ newx, newy, current->g + cell_cost, 0, current };

            // Compute the heuristic for the new state. Diagonal distance is used here.
            new_state.h = MAX(abs(new_state.x - goal->x), abs(new_state.y - goal->y));

            // Check if the new state is in the open list
            bool in_open_list = open_set.find(new_state) != open_set.end();
            if (in_open_list)
            {
              std::cout << "in open list" << std::endl;
              std::cout << "new_state: " << new_state.x << " " << new_state.y << std::endl;
              // If the new state is in the open list, update the cost and parent if necessary
              State existing_state = *open_set.find(new_state);
              if (new_state.g < existing_state.g)
              {
                existing_state.g = new_state.g;
                existing_state.parent = new_state.parent;
              }
            }
            else
            {
              // Add the new state to the open list and the open set
              open_list.push(&new_state);
              open_set.insert(new_state);
            }
          }
        }
      }
    }
    std::cout << "open_list size: " << open_list.size() << std::endl;
  }
}