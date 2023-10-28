/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex>     // For regex and split logic
#include <iostream>  // cout, endl
#include <fstream>   // For reading/writing files
#include <assert.h>

#include <unordered_map>

/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y * XSIZE + X)

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::array;
using std::cout;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief
/// @param filepath
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath)
{
  std::FILE* f = fopen(filepath.c_str(), "r");
  if (f)
  {
  }
  else
  {
    printf("Opening file failed! \n");
    throw runtime_error("Opening map file failed!");
  }
  int height, width;
  if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2)
  {
    throw runtime_error("Invalid loadMap parsing map metadata");
  }

  ////// Go through file and add to m_occupancy
  double* map = new double[height * width];

  double cx, cy, cz;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      char c;
      do
      {
        if (fscanf(f, "%c", &c) != 1)
        {
          throw runtime_error("Invalid parsing individual map data");
        }
      } while (isspace(c));
      if (!(c == '0'))
      {
        map[y + x * width] = 1;  // Note transposed from visual
      }
      else
      {
        map[y + x * width] = 0;
      }
    }
  }
  fclose(f);
  return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim)
{
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
  const std::regex ws_re(delim);
  return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}

double* doubleArrayFromString(string str)
{
  vector<string> vals = split(str, ",");
  double* ans = new double[vals.size()];
  for (int i = 0; i < vals.size(); ++i)
  {
    ans[i] = std::stod(vals[i]);
  }
  return ans;
}

bool equalDoubleArrays(double* v1, double* v2, int size)
{
  for (int i = 0; i < size; ++i)
  {
    if (abs(v1[i] - v2[i]) > 1e-3)
    {
      cout << endl;
      return false;
    }
  }
  return true;
}

typedef struct
{
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int* pY, int x_size, int y_size)
{
  double cellsize = 1.0;
  // take the nearest cell
  *pX = (int)(x / (double)(cellsize));
  if (x < 0)
    *pX = 0;
  if (*pX >= x_size)
    *pX = x_size - 1;

  *pY = (int)(y / (double)(cellsize));
  if (y < 0)
    *pY = 0;
  if (*pY >= y_size)
    *pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t* params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
    params->Y1 = p1x;
    params->X1 = p1y;
    params->Y2 = p2x;
    params->X2 = p2y;
  }
  else
  {
    params->X1 = p1x;
    params->Y1 = p1y;
    params->X2 = p2x;
    params->Y2 = p2y;
  }

  if ((p2x - p1x) * (p2y - p1y) < 0)
  {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX = params->X2 - params->X1;
  params->DeltaY = params->Y2 - params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t* params, int* x, int* y)
{
  if (params->UsingYIndex)
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  }
  else
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

int get_next_point(bresenham_param_t* params)
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size)
{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  // printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  // make sure the line segment is inside the environment
  if (x0 < 0 || x0 >= x_size || x1 < 0 || x1 >= x_size || y0 < 0 || y0 >= y_size || y1 < 0 || y1 >= y_size)
    return 0;

  ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
  ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

  // printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  // iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do
  {
    get_current_point(&params, &nX, &nY);
    if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1)
      return 0;
  } while (get_next_point(&params));

  return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size)
{
  double x0, y0, x1, y1;
  int i;

  // iterate through all the links starting with the base
  x1 = ((double)x_size) / 2.0;
  y1 = 0;
  for (i = 0; i < numofDOFs; i++)
  {
    // compute the corresponding line segment
    x0 = x1;
    y0 = y1;
    x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
    y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

    // check the validity of the corresponding line segment
    if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size))
      return 0;
  }
  return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                    int numofDOFs, double*** plan, int* planlength)
{
  // no plan by default
  *plan = NULL;
  *planlength = 0;

  // for now just do straight interpolation between start and goal checking for the validity of samples

  double distance = 0;
  int i, j;
  for (j = 0; j < numofDOFs; j++)
  {
    if (distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
      distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
  }
  int numofsamples = (int)(distance / (PI / 20));
  if (numofsamples < 2)
  {
    printf("the arm is already at the goal\n");
    return;
  }
  *plan = (double**)malloc(numofsamples * sizeof(double*));
  int firstinvalidconf = 1;
  for (i = 0; i < numofsamples; i++)
  {
    (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
    for (j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = armstart_anglesV_rad[j] +
                      ((double)(i) / (numofsamples - 1)) * (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    }
    if (!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
    {
      firstinvalidconf = 1;
      printf("ERROR: Invalid arm configuration!!!\n");
    }
  }
  *planlength = numofsamples;

  return;
}

bool isValidEdge(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                 int numofDOFs)
{
  // for now just do straight interpolation between start and goal checking for the validity of samples

  double distance = 0;
  int i, j;
  for (j = 0; j < numofDOFs; j++)
  {
    if (distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
      distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
  }
  int numofsamples = (int)(distance / (PI / 20));
  if (numofsamples < 2)
  {
    // printf("the arm is already at the goal\n");
    return true;
  }
  vector<vector<double>> plan_vec = vector<vector<double>>(numofsamples, vector<double>(numofDOFs));
  for (i = 0; i < numofsamples; i++)
  {
    for (j = 0; j < numofDOFs; j++)
    {
      plan_vec[i][j] = armstart_anglesV_rad[j] +
                       ((double)(i) / (numofsamples - 1)) * (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    }
    if (!IsValidArmConfiguration(plan_vec[i].data(), numofDOFs, map, x_size, y_size))
    {
      return false;
    }
  }
  return true;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

struct node
{
  std::vector<double> parent;
};

struct node_hash
{
  std::size_t operator()(const std::vector<double>& n) const
  {
    std::size_t seed = 0;
    // Combine the hash values of all elements in the parent vector
    for (const double& value : n)
    {
      seed ^= std::hash<double>()(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// Function to compute the distance between two configurations
double computeDistance(std::vector<double> config1, std::vector<double> config2)
{
  double dist = 0;
  for (int j = 0; j < config1.size(); j++)
  {
    dist += pow(config1[j] - config2[j], 2);
  }
  return sqrt(dist) / config1.size();
}

// Function to find the nearest node in the tree to a given node
std::vector<double> findNearestNode(std::unordered_map<std::vector<double>, node, node_hash> tree,
                                    std::vector<double> rand_config)
{
  double min_dist = std::numeric_limits<double>::max();
  std::vector<double> nearest_node;
  for (const auto& temp : tree)
  {
    double dist = computeDistance(temp.first, rand_config);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_node = temp.first;
    }
  }
  return nearest_node;
}

std::vector<double> extend(std::vector<double> nearest_node, std::vector<double> rand_config, double step_size)
{
  std::vector<double> new_config(nearest_node.size());
  for (int j = 0; j < nearest_node.size(); j++)
  {
    new_config[j] = nearest_node[j] + step_size * (rand_config[j] - nearest_node[j]) /
                                          sqrt(pow(rand_config[j] - nearest_node[j], 2) + pow(step_size, 2));
  }
  return new_config;
}

static void plannerRRT(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                       int numofDOFs, double*** plan, int* planlength)
{
  int max_iter = 20000;
  double step_size = 0.5;
  // int max_iter = 20000;
  // double step_size = 0.2;
  std::vector<double> armstart_anglesV_rad_vec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> armgoal_anglesV_rad_vec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
  // tree is an unordered_map with key as the node and value as the parent node
  std::unordered_map<std::vector<double>, node, node_hash> tree;
  tree[armstart_anglesV_rad_vec] = node();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);

  for (int iteration = 0; iteration < max_iter; iteration++)
  {
    vector<double> rand_config(numofDOFs);
    for (int j = 0; j < numofDOFs; j++)
    {
      rand_config[j] = dis(gen) * 2 * PI;
      // rand_config[j] = ((double)rand() / RAND_MAX) * 2 * PI;
    }

    // Find nearest node
    std::vector<double> nearest_node = findNearestNode(tree, rand_config);

    // Extend towards random node
    std::vector<double> new_config = extend(nearest_node, rand_config, step_size);

    // Check if new node is valid
    if (IsValidArmConfiguration(new_config.data(), numofDOFs, map, x_size, y_size))
    {
      if (isValidEdge(map, x_size, y_size, nearest_node.data(), new_config.data(), numofDOFs))
      {
        tree[new_config].parent = nearest_node;
        // Check if new node is close to goal
        double dist = computeDistance(new_config, armgoal_anglesV_rad_vec);
        if (dist < step_size &&
            isValidEdge(map, x_size, y_size, new_config.data(), armgoal_anglesV_rad_vec.data(), numofDOFs))
        {
          tree[armgoal_anglesV_rad_vec].parent = new_config;
          // Build the plan by backtracking through the tree
          std::vector<std::vector<double>> plan_vec;
          std::vector<double> current_node = armgoal_anglesV_rad_vec;
          while (current_node != armstart_anglesV_rad_vec)
          {
            plan_vec.push_back(current_node);
            current_node = tree[current_node].parent;
          }
          plan_vec.push_back(armstart_anglesV_rad_vec);
          std::reverse(plan_vec.begin(), plan_vec.end());
          *planlength = plan_vec.size();
          *plan = (double**)malloc((*planlength) * sizeof(double*));
          for (int i = 0; i < *planlength; i++)
          {
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
            for (int j = 0; j < numofDOFs; j++)
            {
              (*plan)[i][j] = plan_vec[i][j];
            }
          }
          return;
        }
      }
    }
  }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

// connect function keeps calling the extend function to connect the nearest node in the goal tree to the current node
// until the current node is reached or there is a collision
bool connect(std::unordered_map<std::vector<double>, node, node_hash>& tree, std::vector<double> nearest_node,
             std::vector<double> new_node, double step_size, int numofDOFs, double* map, int x_size, int y_size)
{
  while (true)
  {
    std::vector<double> new_config = extend(nearest_node, new_node, step_size);
    if (!IsValidArmConfiguration(new_config.data(), numofDOFs, map, x_size, y_size))
    {
      return false;
    }
    if (!isValidEdge(map, x_size, y_size, nearest_node.data(), new_config.data(), numofDOFs))
    {
      return false;
    }
    tree[new_config].parent = nearest_node;
    if (computeDistance(new_config, new_node) < step_size)
    {
      return true;
    }
    nearest_node = new_config;
  }
}

static void plannerRRTConnect(double* map, int x_size, int y_size, double* armstart_anglesV_rad,
                              double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength)
{
  int max_iter = 10000;
  double step_size = 0.5;
  // int max_iter = 20000;
  // double step_size = 0.2;
  std::vector<double> armstart_anglesV_rad_vec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> armgoal_anglesV_rad_vec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
  // tree is an unordered_map with key as the node and value as the parent node
  std::unordered_map<std::vector<double>, node, node_hash> tree_start;
  tree_start[armstart_anglesV_rad_vec] = node();
  std::unordered_map<std::vector<double>, node, node_hash> tree_goal;
  tree_goal[armgoal_anglesV_rad_vec] = node();  // FIXME: This might not be needed

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);
  int swap = 0;
  for (int iteration = 0; iteration < max_iter; iteration++)
  {
    vector<double> rand_config(numofDOFs);
    for (int j = 0; j < numofDOFs; j++)
    {
      rand_config[j] = dis(gen) * 2 * PI;
      // rand_config[j] = ((double)rand() / RAND_MAX) * 2 * PI;
    }

    // Find nearest node
    std::vector<double> nearest_node = findNearestNode(tree_start, rand_config);
    // Extend towards random node
    vector<double> new_config = extend(nearest_node, rand_config, step_size);
    if (IsValidArmConfiguration(new_config.data(), numofDOFs, map, x_size, y_size))
    {
      if (isValidEdge(map, x_size, y_size, nearest_node.data(), new_config.data(), numofDOFs))
      {
        tree_start[new_config].parent = nearest_node;
        std::vector<double> nearest_node_goal = findNearestNode(tree_goal, new_config);
        // connect the nearest node in the goal tree to the current node
        if (connect(tree_goal, nearest_node_goal, new_config, step_size, numofDOFs, map, x_size, y_size))
        {
          tree_goal[new_config].parent = nearest_node_goal;
          // Build the plan by backtracking through the tree
          std::vector<std::vector<double>> plan_vec;
          std::vector<std::vector<double>> plan_vec_goal;
          std::vector<double> current_node = new_config;
          while (current_node != tree_start[armstart_anglesV_rad_vec].parent)
          {
            plan_vec.push_back(current_node);
            current_node = tree_start[current_node].parent;
          }
          std::reverse(plan_vec.begin(), plan_vec.end());
          current_node = tree_goal[new_config].parent;
          while (current_node != tree_goal[armgoal_anglesV_rad_vec].parent)
          {
            plan_vec_goal.push_back(current_node);
            current_node = tree_goal[current_node].parent;
          }
          plan_vec.insert(plan_vec.end(), plan_vec_goal.begin(), plan_vec_goal.end());
          *planlength = plan_vec.size();
          *plan = (double**)malloc((*planlength) * sizeof(double*));
          for (int i = 0; i < *planlength; i++)
          {
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
            for (int j = 0; j < numofDOFs; j++)
            {
              (*plan)[i][j] = plan_vec[i][j];
            }
          }

          if (swap % 2 == 1)
          {
            std::reverse(*plan, *plan + *planlength);
          }
          return;
        }
        std::swap(tree_start, tree_goal);
        swap++;
      }
    }
  }
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

struct node_star
{
  std::vector<double> parent;
  double cost;
};

typedef std::shared_ptr<node_star> node_star_ptr;

// Function to find the nearest node in the tree to a given node
std::vector<double> findNearestNodestar(std::unordered_map<std::vector<double>, node_star_ptr, node_hash> tree,
                                        std::vector<double> rand_config)
{
  double min_dist = std::numeric_limits<double>::max();
  std::vector<double> nearest_node;
  for (const auto& temp : tree)
  {
    double dist = computeDistance(temp.first, rand_config);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_node = temp.first;
    }
  }
  return nearest_node;
}

// FUnction to find all the nodes in the tree that are within a certain neighborhood radius of the new node
std::vector<std::vector<double>> findNear(std::unordered_map<std::vector<double>, node_star, node_hash> tree,
                                          std::vector<double> new_node, double neighborhood_radius)
{
  std::vector<std::vector<double>> near_nodes;
  for (const auto& temp : tree)
  {
    double dist = computeDistance(temp.first, new_node);
    if (dist < neighborhood_radius)
    {
      near_nodes.push_back(temp.first);
    }
  }
  return near_nodes;
}

bool isValidEdgestar(double* map, int x_size, int y_size, const double* armstart_anglesV_rad,
                     const double* armgoal_anglesV_rad, int numofDOFs)
{
  // for now just do straight interpolation between start and goal checking for the validity of samples

  double distance = 0;
  int i, j;
  for (j = 0; j < numofDOFs; j++)
  {
    if (distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
      distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
  }
  int numofsamples = (int)(distance / (PI / 20));
  if (numofsamples < 2)
  {
    // printf("the arm is already at the goal\n");
    return true;
  }
  vector<vector<double>> plan_vec = vector<vector<double>>(numofsamples, vector<double>(numofDOFs));
  for (i = 0; i < numofsamples; i++)
  {
    for (j = 0; j < numofDOFs; j++)
    {
      plan_vec[i][j] = armstart_anglesV_rad[j] +
                       ((double)(i) / (numofsamples - 1)) * (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    }
    if (!IsValidArmConfiguration(plan_vec[i].data(), numofDOFs, map, x_size, y_size))
    {
      return false;
    }
  }
  return true;
}

// Function to rewrire the tree with the new node for all the nodes that are within a certain neighborhood radius
// take the neighborhood radius as a parameter
void rewire(std::unordered_map<std::vector<double>, node_star_ptr, node_hash>& tree, std::vector<double> new_node,
            double step_size, int numofDOFs, double* map, int x_size, int y_size, double neighborhood_radius)
// ---------------------------------------------------------------------------------------------------
// TODO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!NEEDS TO BE OPTIMIZED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ---------------------------------------------------------------------------------------------------
{
  for (const auto& temp : tree)
  {
    // if temp is the parent of the new node, then skip
    if (temp.first == tree[new_node]->parent)
    {
      continue;
    }
    double dist = computeDistance(temp.first, new_node);
    if (dist < neighborhood_radius)
    {
      if (isValidEdgestar(map, x_size, y_size, temp.first.data(), new_node.data(), numofDOFs))
      {
        auto new_node_cost = tree[temp.first]->cost + dist;
        if (tree[new_node]->cost > new_node_cost)
        {
          tree[new_node]->parent = temp.first;
          tree[new_node]->cost = new_node_cost;
        }
        auto temp_node_cost = tree[new_node]->cost + dist;
        if (tree[temp.first]->cost > temp_node_cost)
        {
          tree[temp.first]->parent = new_node;
          tree[temp.first]->cost = temp_node_cost;
        }
      }
    }
  }
}

static void plannerRRTStar(double* map, int x_size, int y_size, double* armstart_anglesV_rad,
                           double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength)
{
  int max_iter = 10000;
  double step_size = 0.2;
  std::vector<double> armstart_anglesV_rad_vec(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  std::vector<double> armgoal_anglesV_rad_vec(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
  // tree is an unordered_map with key as the node and value as the parent node
  std::unordered_map<std::vector<double>, node_star_ptr, node_hash> tree;
  tree[armstart_anglesV_rad_vec] = std::make_shared<node_star>();
  tree[armstart_anglesV_rad_vec]->cost = 0;
  std::vector<double> goal_parent;
  double goal_cost = std::numeric_limits<double>::max();

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);

  for (int iteration = 0; iteration < max_iter; iteration++)
  {
    vector<double> rand_config(numofDOFs);
    for (int j = 0; j < numofDOFs; j++)
    {
      rand_config[j] = dis(gen) * 2 * PI;
      // rand_config[j] = ((double)rand() / RAND_MAX) * 2 * PI;
    }

    // Find nearest node
    std::vector<double> nearest_node = findNearestNodestar(tree, rand_config);

    // Extend towards random node
    std::vector<double> new_config = extend(nearest_node, rand_config, step_size);

    // Check if new node is valid
    if (IsValidArmConfiguration(new_config.data(), numofDOFs, map, x_size, y_size))
    {
      if (isValidEdgestar(map, x_size, y_size, nearest_node.data(), new_config.data(), numofDOFs))
      {
        tree[new_config] = std::make_shared<node_star>();
        tree[new_config]->parent = nearest_node;
        tree[new_config]->cost =
            tree[nearest_node]->cost +
            computeDistance(nearest_node, new_config);  // TODO: This distance has already
                                                        // been computed in the findNearestNodestar function
        // call the rewire function to rewire the tree with the new node
        int neighborhood_radius = 10 * step_size / numofDOFs;
        rewire(tree, new_config, step_size, numofDOFs, map, x_size, y_size, neighborhood_radius);
        // Check if new node is close to goal
        double dist = computeDistance(new_config, armgoal_anglesV_rad_vec);
        if ((tree[new_config]->cost + dist < goal_cost) &&
            (isValidEdgestar(map, x_size, y_size, new_config.data(), armgoal_anglesV_rad_vec.data(), numofDOFs)))
        {
          goal_parent = new_config;
          goal_cost = tree[new_config]->cost + dist;
        }
      }
    }
  }
  tree[armgoal_anglesV_rad_vec] = std::make_shared<node_star>();
  tree[armgoal_anglesV_rad_vec]->parent = goal_parent;
  tree[armgoal_anglesV_rad_vec]->cost = goal_cost;
  // Build the plan by backtracking through the tree
  std::vector<std::vector<double>> plan_vec;
  std::vector<double> current_node = armgoal_anglesV_rad_vec;
  while (current_node != armstart_anglesV_rad_vec)
  {
    plan_vec.push_back(current_node);
    current_node = tree[current_node]->parent;
  }
  plan_vec.push_back(armstart_anglesV_rad_vec);
  std::reverse(plan_vec.begin(), plan_vec.end());
  *planlength = plan_vec.size();
  *plan = (double**)malloc((*planlength) * sizeof(double*));
  for (int i = 0; i < *planlength; i++)
  {
    (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
    for (int j = 0; j < numofDOFs; j++)
    {
      (*plan)[i][j] = plan_vec[i][j];
    }
  }
  return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                       int numofDOFs, double*** plan, int* planlength)
{
  /* TODO: Replace with your implementation */
  planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos,
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv)
{
  double* map;
  int x_size, y_size;

  tie(map, x_size, y_size) = loadMap(argv[1]);
  const int numOfDOFs = std::stoi(argv[2]);
  double* startPos = doubleArrayFromString(argv[3]);
  double* goalPos = doubleArrayFromString(argv[4]);
  int whichPlanner = std::stoi(argv[5]);
  string outputFile = argv[6];

  if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) ||
      !IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size))
  {
    throw runtime_error("Invalid start or goal configuration!\n");
  }

  ///////////////////////////////////////
  //// Feel free to modify anything below. Be careful modifying anything above.

  double** plan = NULL;
  int planlength = 0;

  // Call the corresponding planner function
  if (whichPlanner == PRM)
  {
    plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRT)
  {
    plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRTCONNECT)
  {
    plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else if (whichPlanner == RRTSTAR)
  {
    plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }
  else
  {
    planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
  }

  //// Feel free to modify anything above.
  //// If you modify something below, please change it back afterwards as my
  //// grading script will not work and you will recieve a 0.
  ///////////////////////////////////////

  // Your solution's path should start with startPos and end with goalPos
  if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || !equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs))
  {
    throw std::runtime_error("Start or goal position not matching");
  }

  /** Saves the solution to output file
   * Do not modify the output log file output format as it is required for visualization
   * and for grading.
   */
  std::ofstream m_log_fstream;
  m_log_fstream.open(outputFile, std::ios::trunc);  // Creates new or replaces existing file
  if (!m_log_fstream.is_open())
  {
    throw std::runtime_error("Cannot open file");
  }
  m_log_fstream << argv[1] << endl;  // Write out map name first
  /// Then write out all the joint angles in the plan sequentially
  for (int i = 0; i < planlength; ++i)
  {
    for (int k = 0; k < numOfDOFs; ++k)
    {
      m_log_fstream << plan[i][k] << ",";
    }
    m_log_fstream << endl;
  }
}
