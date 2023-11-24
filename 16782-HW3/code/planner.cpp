#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <unordered_map>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
  string predicate;
  list<string> arg_values;
  bool truth = true;

public:
  GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
  {
    this->predicate = predicate;
    this->truth = truth;  // fixed
    for (string l : arg_values)
    {
      this->arg_values.push_back(l);
    }
  }

  GroundedCondition(const GroundedCondition& gc)
  {
    this->predicate = gc.predicate;
    this->truth = gc.truth;  // fixed
    for (string l : gc.arg_values)
    {
      this->arg_values.push_back(l);
    }
  }

  string get_predicate() const
  {
    return this->predicate;
  }
  list<string> get_arg_values() const
  {
    return this->arg_values;
  }

  bool get_truth() const
  {
    return this->truth;
  }

  friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
  {
    os << pred.toString() << " ";
    return os;
  }

  bool operator==(const GroundedCondition& rhs) const
  {
    if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
      return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
    {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }

    if (this->truth != rhs.get_truth())  // fixed
      return false;

    return true;
  }

  string toString() const
  {
    string temp = "";
    temp += this->predicate;
    temp += "(";
    for (string l : this->arg_values)
    {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }
};

struct GroundedConditionComparator
{
  bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
  {
    return lhs == rhs;
  }
};

struct GroundedConditionHasher
{
  size_t operator()(const GroundedCondition& gcond) const
  {
    return hash<string>{}(gcond.toString());
  }
};

class Condition
{
private:
  string predicate;
  list<string> args;
  bool truth;

public:
  Condition(string pred, list<string> args, bool truth)
  {
    this->predicate = pred;
    this->truth = truth;
    for (string ar : args)
    {
      this->args.push_back(ar);
    }
  }

  string get_predicate() const
  {
    return this->predicate;
  }

  list<string> get_args() const
  {
    return this->args;
  }

  bool get_truth() const
  {
    return this->truth;
  }

  friend ostream& operator<<(ostream& os, const Condition& cond)
  {
    os << cond.toString() << " ";
    return os;
  }

  bool operator==(const Condition& rhs) const  // fixed
  {
    if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
      return false;

    auto lhs_it = this->args.begin();
    auto rhs_it = rhs.args.begin();

    while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
    {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }

    if (this->truth != rhs.get_truth())
      return false;

    return true;
  }

  string toString() const
  {
    string temp = "";
    if (!this->truth)
      temp += "!";
    temp += this->predicate;
    temp += "(";
    for (string l : this->args)
    {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }
};

struct ConditionComparator
{
  bool operator()(const Condition& lhs, const Condition& rhs) const
  {
    return lhs == rhs;
  }
};

struct ConditionHasher
{
  size_t operator()(const Condition& cond) const
  {
    return hash<string>{}(cond.toString());
  }
};

class Action
{
private:
  string name;
  list<string> args;
  unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
  unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
  Action(string name, list<string> args, unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
         unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
  {
    this->name = name;
    for (string l : args)
    {
      this->args.push_back(l);
    }
    for (Condition pc : preconditions)
    {
      this->preconditions.insert(pc);
    }
    for (Condition pc : effects)
    {
      this->effects.insert(pc);
    }
  }
  string get_name() const
  {
    return this->name;
  }
  list<string> get_args() const
  {
    return this->args;
  }
  unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
  {
    return this->preconditions;
  }
  unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
  {
    return this->effects;
  }

  bool operator==(const Action& rhs) const
  {
    if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
      return false;

    return true;
  }

  friend ostream& operator<<(ostream& os, const Action& ac)
  {
    os << ac.toString() << endl;
    os << "Precondition: ";
    for (Condition precond : ac.get_preconditions())
      os << precond;
    os << endl;
    os << "Effect: ";
    for (Condition effect : ac.get_effects())
      os << effect;
    os << endl;
    return os;
  }

  string toString() const
  {
    string temp = "";
    temp += this->get_name();
    temp += "(";
    for (string l : this->get_args())
    {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }
};

struct ActionComparator
{
  bool operator()(const Action& lhs, const Action& rhs) const
  {
    return lhs == rhs;
  }
};

struct ActionHasher
{
  size_t operator()(const Action& ac) const
  {
    return hash<string>{}(ac.get_name());
  }
};

class Env
{
private:
  unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
  unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
  unordered_set<Action, ActionHasher, ActionComparator> actions;
  unordered_set<string> symbols;

  friend list<GroundedAction> planner(Env* env);  // planner function can access private variables of Env

public:
  void remove_initial_condition(GroundedCondition gc)
  {
    this->initial_conditions.erase(gc);
  }
  void add_initial_condition(GroundedCondition gc)
  {
    this->initial_conditions.insert(gc);
  }
  void add_goal_condition(GroundedCondition gc)
  {
    this->goal_conditions.insert(gc);
  }
  void remove_goal_condition(GroundedCondition gc)
  {
    this->goal_conditions.erase(gc);
  }
  void add_symbol(string symbol)
  {
    symbols.insert(symbol);
  }
  void add_symbols(list<string> symbols)
  {
    for (string l : symbols)
      this->symbols.insert(l);
  }
  void add_action(Action action)
  {
    this->actions.insert(action);
  }

  Action get_action(string name)
  {
    for (Action a : this->actions)
    {
      if (a.get_name() == name)
        return a;
    }
    throw runtime_error("Action " + name + " not found!");
  }
  unordered_set<string> get_symbols() const
  {
    return this->symbols;
  }

  friend ostream& operator<<(ostream& os, const Env& w)
  {
    os << "***** Environment *****" << endl << endl;
    os << "Symbols: ";
    for (string s : w.get_symbols())
      os << s + ",";
    os << endl;
    os << "Initial conditions: ";
    for (GroundedCondition s : w.initial_conditions)
      os << s;
    os << endl;
    os << "Goal conditions: ";
    for (GroundedCondition g : w.goal_conditions)
      os << g;
    os << endl;
    os << "Actions:" << endl;
    for (Action g : w.actions)
      os << g << endl;
    cout << "***** Environment Created! *****" << endl;
    return os;
  }
};

class GroundedAction
{
private:
  string name;
  list<string> arg_values;

public:
  GroundedAction(string name, list<string> arg_values)
  {
    this->name = name;
    for (string ar : arg_values)
    {
      this->arg_values.push_back(ar);
    }
  }

  string get_name() const
  {
    return this->name;
  }

  list<string> get_arg_values() const
  {
    return this->arg_values;
  }

  bool operator==(const GroundedAction& rhs) const
  {
    if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
      return false;

    auto lhs_it = this->arg_values.begin();
    auto rhs_it = rhs.arg_values.begin();

    while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
    {
      if (*lhs_it != *rhs_it)
        return false;
      ++lhs_it;
      ++rhs_it;
    }
    return true;
  }

  friend ostream& operator<<(ostream& os, const GroundedAction& gac)
  {
    os << gac.toString() << " ";
    return os;
  }

  string toString() const
  {
    string temp = "";
    temp += this->name;
    temp += "(";
    for (string l : this->arg_values)
    {
      temp += l + ",";
    }
    temp = temp.substr(0, temp.length() - 1);
    temp += ")";
    return temp;
  }
};

list<string> parse_symbols(string symbols_str)
{
  list<string> symbols;
  size_t pos = 0;
  string delimiter = ",";
  while ((pos = symbols_str.find(delimiter)) != string::npos)
  {
    string symbol = symbols_str.substr(0, pos);
    symbols_str.erase(0, pos + delimiter.length());
    symbols.push_back(symbol);
  }
  symbols.push_back(symbols_str);
  return symbols;
}

Env* create_env(char* filename)
{
  ifstream input_file(filename);
  Env* env = new Env();
  regex symbolStateRegex("symbols:", regex::icase);
  regex symbolRegex("([a-zA-Z0-9_, ]+) *");
  regex initialConditionRegex("initialconditions:(.*)", regex::icase);
  regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
  regex goalConditionRegex("goalconditions:(.*)", regex::icase);
  regex actionRegex("actions:", regex::icase);
  regex precondRegex("preconditions:(.*)", regex::icase);
  regex effectRegex("effects:(.*)", regex::icase);
  int parser = SYMBOLS;

  unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
  unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
  string action_name;
  string action_args;

  string line;
  if (input_file.is_open())
  {
    while (getline(input_file, line))
    {
      string::iterator end_pos = remove(line.begin(), line.end(), ' ');
      line.erase(end_pos, line.end());

      if (line == "")
        continue;

      if (parser == SYMBOLS)
      {
        smatch results;
        if (regex_search(line, results, symbolStateRegex))
        {
          line = line.substr(8);
          sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
          sregex_token_iterator end;

          env->add_symbols(parse_symbols(iter->str()));  // fixed

          parser = INITIAL;
        }
        else
        {
          cout << "Symbols are not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == INITIAL)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, initialConditionRegex))
        {
          const std::vector<int> submatches = { 1, 2 };
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex, submatches);
          sregex_token_iterator end;

          while (iter != end)
          {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            if (predicate[0] == '!')
            {
              env->remove_initial_condition(GroundedCondition(predicate.substr(1), parse_symbols(args)));
            }
            else
            {
              env->add_initial_condition(GroundedCondition(predicate, parse_symbols(args)));
            }
          }

          parser = GOAL;
        }
        else
        {
          cout << "Initial conditions not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == GOAL)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, goalConditionRegex))
        {
          const std::vector<int> submatches = { 1, 2 };
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex, submatches);
          sregex_token_iterator end;

          while (iter != end)
          {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            if (predicate[0] == '!')
            {
              env->remove_goal_condition(GroundedCondition(predicate.substr(1), parse_symbols(args)));
            }
            else
            {
              env->add_goal_condition(GroundedCondition(predicate, parse_symbols(args)));
            }
          }

          parser = ACTIONS;
        }
        else
        {
          cout << "Goal conditions not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == ACTIONS)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, actionRegex))
        {
          parser = ACTION_DEFINITION;
        }
        else
        {
          cout << "Actions not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == ACTION_DEFINITION)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, conditionRegex))
        {
          const std::vector<int> submatches = { 1, 2 };
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex, submatches);
          sregex_token_iterator end;
          // name
          action_name = iter->str();
          iter++;
          // args
          action_args = iter->str();
          iter++;

          parser = ACTION_PRECONDITION;
        }
        else
        {
          cout << "Action not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == ACTION_PRECONDITION)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, precondRegex))
        {
          const std::vector<int> submatches = { 1, 2 };
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex, submatches);
          sregex_token_iterator end;

          while (iter != end)
          {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            bool truth;

            if (predicate[0] == '!')
            {
              predicate = predicate.substr(1);
              truth = false;
            }
            else
            {
              truth = true;
            }

            Condition precond(predicate, parse_symbols(args), truth);
            preconditions.insert(precond);
          }

          parser = ACTION_EFFECT;
        }
        else
        {
          cout << "Precondition not specified correctly." << endl;
          throw;
        }
      }
      else if (parser == ACTION_EFFECT)
      {
        const char* line_c = line.c_str();
        if (regex_match(line_c, effectRegex))
        {
          const std::vector<int> submatches = { 1, 2 };
          sregex_token_iterator iter(line.begin(), line.end(), conditionRegex, submatches);
          sregex_token_iterator end;

          while (iter != end)
          {
            // name
            string predicate = iter->str();
            iter++;
            // args
            string args = iter->str();
            iter++;

            bool truth;

            if (predicate[0] == '!')
            {
              predicate = predicate.substr(1);
              truth = false;
            }
            else
            {
              truth = true;
            }

            Condition effect(predicate, parse_symbols(args), truth);
            effects.insert(effect);
          }

          env->add_action(Action(action_name, parse_symbols(action_args), preconditions, effects));

          preconditions.clear();
          effects.clear();
          parser = ACTION_DEFINITION;
        }
        else
        {
          cout << "Effects not specified correctly." << endl;
          throw;
        }
      }
    }
    input_file.close();
  }

  else
    cout << "Unable to open file";

  return env;
}

struct Node
{
  unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;
  std::shared_ptr<Node> parent;
  // GroundedAction action;
  double g_cost;
  double heuristic;

  // Node(Node* parent, unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state,
  // double g_cost, double heuristic)
  // {
  //   this->parent = parent;
  //   // this->action = action;
  //   this->state = state;
  //   this->g_cost = g_cost;
  //   this->heuristic = heuristic;
  // }

  // bool operator==(const Node& rhs) const
  // {
  //   if (this->action != rhs.action || this->state.size() != rhs.state.size())
  //     return false;

  //   auto lhs_it = this->state.begin();
  //   auto rhs_it = rhs.state.begin();

  //   while (lhs_it != this->state.end() && rhs_it != rhs.state.end())
  //   {
  //     if (*lhs_it != *rhs_it)
  //       return false;
  //     ++lhs_it;
  //     ++rhs_it;
  //   }
  //   return true;
  // }

  // friend ostream& operator<<(ostream& os, const Node& node)
  // {
  //   os << node.toString() << " ";
  //   return os;
  // }

  // string toString() const
  // {
  //   string temp = "";
  //   temp += this->action.toString();
  //   temp += " ";
  //   temp += to_string(this->g_cost);
  //   temp += " ";
  //   temp += to_string(this->heuristic);
  //   return temp;
  // }
};

struct NodeCostComparator
{
  bool operator()(const Node& lhs, const Node& rhs) const
  {
    return lhs.g_cost + lhs.heuristic > rhs.g_cost + rhs.heuristic;
  }
};

struct NodeStateComparator
{
  bool operator()(const Node& lhs, const Node& rhs) const
  {
    return lhs.state == rhs.state;
  }
};

struct NodeHasher  // TODO: get a better hash function
{
  size_t operator()(const Node& node) const
  {
    string temp = "";
    for (GroundedCondition gc : node.state)
    {
      temp += gc.toString();
    }
    return hash<string>{}(temp);
  }
};

size_t compute_heuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state,
                         unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal)
{
  size_t heuristic = 0;
  for (GroundedCondition gc : goal)
  {
    if (state.find(gc) == state.end())
      heuristic++;
  }
  return heuristic;
}

void combine_util(const unordered_set<string>& symbols, int combination_size, list<string>& temp,
                  list<list<string>>& combinations, unordered_set<string>::const_iterator it)
{
  // Base condition: if the combination size is met
  if (temp.size() == combination_size)
  {
    combinations.push_back(temp);
    return;
  }

  // Stop if no more elements can be added
  if (it == symbols.end())
  {
    return;
  }

  // Include the current symbol and move to the next
  temp.push_back(*it);
  combine_util(symbols, combination_size, temp, combinations, next(it));

  // Exclude the current symbol and move to the next
  temp.pop_back();
  combine_util(symbols, combination_size, temp, combinations, next(it));
}

list<list<string>> generate_combinations(const unordered_set<string>& symbols, int combination_size)
{
  list<list<string>> combinations;
  list<string> temp;
  combine_util(symbols, combination_size, temp, combinations, symbols.begin());
  return combinations;
}

bool is_precondition_met(
    const Condition& precondition,
    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& current_state,
    const GroundedAction& grounded_action)
{
  // Replace action arguments with grounded action arguments
  list<string> args = precondition.get_args();
  list<string> grounded_args = grounded_action.get_arg_values();
  auto it_ground = grounded_args.begin();
  for (auto& arg : args)
  {
    if (arg[0] == '?')  // Assuming arguments in actions start with '?'
    {
      arg = *it_ground;
      ++it_ground;
    }
  }

  GroundedCondition gc(precondition.get_predicate(), args, precondition.get_truth());

  // Check if the grounded condition is in the current state
  return current_state.find(gc) != current_state.end();
}

void generateArgumentCombinations(vector<list<string>>& result, const list<string>& args)
{
  if (args.empty())
  {
    // If there are no arguments, there's only one combination (empty).
    result.push_back(list<string>());
    return;
  }

  // Convert the list of arguments to a vector for easier iteration.
  vector<string> argsVector(args.begin(), args.end());

  // Recursive function to generate combinations.
  function<void(size_t, list<string>)> generate = [&](size_t index, list<string> currentCombination) {
    if (index == argsVector.size())
    {
      // We've reached the end of the arguments, add the current combination to the result.
      result.push_back(currentCombination);
      return;
    }

    // Include the current argument in the combination.
    currentCombination.push_back(argsVector[index]);
    generate(index + 1, currentCombination);

    // Exclude the current argument from the combination.
    currentCombination.pop_back();
    generate(index + 1, currentCombination);
  };

  // Start the recursive generation process.
  generate(0, list<string>());
}

bool all_preconditions_met(
    const unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& current_state,
    const GroundedAction& grounded_action)
{
  for (const Condition& precondition : preconditions)
  {
    if (!is_precondition_met(precondition, current_state, grounded_action))
    {
      return false;
    }
  }
  return true;
}

list<GroundedAction> planner(Env* env)
{
  // Create an open list (priority queue) for states to explore
  std::priority_queue<Node, std::vector<Node>, NodeCostComparator> open_list;
  // Create a closed list (unordered set) for states already explored
  std::unordered_set<Node, NodeHasher, NodeStateComparator> closed_list;
  // Create a set to store all open nodes
  std::unordered_set<Node, NodeHasher, NodeStateComparator> all_open_nodes;

  // Create a node for the initial state
  Node start_node;
  start_node.state = env->initial_conditions;
  start_node.parent = nullptr;
  start_node.g_cost = 0;
  start_node.heuristic = compute_heuristic(start_node.state, env->goal_conditions);

  // Add the initial state to the open list
  open_list.push(start_node);
  all_open_nodes.insert(start_node);
  closed_list.insert(start_node);

  // While the open list is not empty
  while (!open_list.empty())
  {
    // Pop the node with the lowest cost
    Node current_node = open_list.top();
    open_list.pop();
    closed_list.insert(current_node);
    all_open_nodes.insert(current_node);

    // If the current node is a goal state
    if (current_node.state == env->goal_conditions)
    {
      cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Goal state "
              "found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
           << endl;
      // Return the path to the goal
      list<GroundedAction> path;
      // list of all the states in the path
      list<unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>> path_states;

      while (current_node.parent != nullptr)
      {
        // path.push_front(current_node.action); // FIXME: action is not stored in the node
        path_states.push_front(current_node.state);
        current_node = *current_node.parent;
      }
      path_states.push_front(current_node.state);
      cout << "states in the path: " << endl;
      for (unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state : path_states)
      {
        for (GroundedCondition gc : state)
        {
          cout << gc << endl;
        }
        cout << endl;
      }
      return path;
    }

    // For each action in the environment
    for (Action action : env->actions)
    {
      // Generate all combinations of arguments
      list<list<string>> combinations = generate_combinations(env->get_symbols(), action.get_args().size());

      for (const list<string>& args : combinations)
      {
        GroundedAction grounded_action(action.get_name(), args);
        // Check if preconditions are met
        if (all_preconditions_met(action.get_preconditions(), current_node.state, grounded_action))
        {
          cout << "Action " << grounded_action << " is applicable" << endl;
          // Apply the action to the current state // TODO: make this a function
          unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state =
              current_node.state;
          for (Condition effect : action.get_effects())
          {
            GroundedCondition grounded_effect(effect.get_predicate(), args, effect.get_truth());
            if (grounded_effect.get_truth())
            {
              new_state.insert(grounded_effect);
            }
            else
            {
              new_state.erase(grounded_effect);
            }
          }

          // Create a new node for the action
          Node new_node;
          new_node.state = new_state;
          new_node.parent = make_shared<Node>(current_node);
          // new_node.action = grounded_action; // FIXME: action is not stored in the node
          new_node.g_cost = current_node.g_cost + 1;
          new_node.heuristic = compute_heuristic(new_node.state, env->goal_conditions);

          // If the new node is not in the closed list
          if (closed_list.find(new_node) == closed_list.end())
          {
            // If the new node is not in the open list
            if (all_open_nodes.find(new_node) == all_open_nodes.end())
            {
              // Add the new node to the open list
              open_list.push(new_node);
              all_open_nodes.insert(new_node);
            }
          }
        }
        cout << "Action " << grounded_action << " is not applicable" << endl;
      }
    }
  }
  return list<GroundedAction>();
}

int main(int argc, char* argv[])
{
  // DO NOT CHANGE THIS FUNCTION
  char* filename = (char*)("example.txt");
  if (argc > 1)
    filename = argv[1];

  cout << "Environment: " << filename << endl << endl;
  Env* env = create_env(filename);
  if (print_status)
  {
    cout << *env;
  }

  list<GroundedAction> actions = planner(env);

  cout << "\nPlan: " << endl;
  for (GroundedAction gac : actions)
  {
    cout << gac << endl;
  }

  return 0;
}