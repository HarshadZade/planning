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
#include <chrono>
#include <limits.h>

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
size_t min_literals_added = 0;
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
  friend int get_min_literals_added(const Env* env);

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
  GroundedAction action;  // Store the action leading to this node
  double g_cost;
  double heuristic;

  Node() = default;

  Node(const GroundedAction& action,
       const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& state,
       const std::shared_ptr<Node>& parent, double g_cost, double heuristic)
    : action(action), state(state), parent(parent), g_cost(g_cost), heuristic(heuristic)
  {
  }
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

struct NodeHasher
{
  size_t operator()(const Node& node) const
  {
    string temp = "";
    for (GroundedCondition gc : node.state)
    {
      temp += gc.toString();
    }
    // reorder the temp string to make sure the hash is the same for the same state
    sort(temp.begin(), temp.end());
    return hash<string>{}(temp);
  }
};

int get_min_literals_added(const Env* env)
{
  int min_literals = INT_MAX;  // Start with the maximum possible integer value

  for (const Action& action : env->actions)
  {
    int literals_added = 0;
    for (const Condition& effect : action.get_effects())
    {
      if (effect.get_truth())
      {  // Only count positive effects
        literals_added++;
      }
    }
    if (literals_added > 0 && literals_added < min_literals)
    {
      min_literals = literals_added;
    }
  }

  return min_literals == INT_MAX ? 0 : min_literals;  // Return 0 if no action adds any literals
}

size_t compute_heuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state,
                         unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal)
{
  size_t heuristic = 0;
  for (GroundedCondition gc : goal)
  {
    if (state.find(gc) == state.end())
      heuristic++;
  }
  return heuristic / min_literals_added;
  // return heuristic;
  // return 0; // To check if the algorithm works without heuristic
}

void permute_util(const unordered_set<string>& symbols, int permutation_size, list<string>& temp,
                  list<list<string>>& permutations, unordered_set<string>& used)
{
  // Base condition: if the permutation size is met
  if (temp.size() == permutation_size)
  {
    permutations.push_back(temp);
    return;
  }

  for (const auto& symbol : symbols)
  {
    // Check if the symbol has already been used in this permutation
    if (used.find(symbol) != used.end())
    {
      continue;
    }

    // Include the current symbol
    temp.push_back(symbol);
    used.insert(symbol);

    // Recursive call
    permute_util(symbols, permutation_size, temp, permutations, used);

    // Remove the symbol from the used set and exclude it from the current permutation
    used.erase(symbol);
    temp.pop_back();
  }
}

list<list<string>> generate_permutations(const unordered_set<string>& symbols, int permutation_size)
{
  list<list<string>> permutations;
  list<string> temp;
  unordered_set<string> used;
  permute_util(symbols, permutation_size, temp, permutations, used);
  return permutations;
}
void print_arg_list(const list<string>& args)
{
  for (const auto& arg : args)
  {
    cout << arg << " ";
  }
}

bool are_all_preconditions_met(
    const Action& action, const list<string>& grounded_args,
    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& current_state)
{
  const list<string>& action_args = action.get_args();  // Placeholders from the action

  // Iterate over each precondition of the action
  for (const Condition& precondition : action.get_preconditions())
  {
    list<string> replaced_args;  // List to store replaced arguments

    // Replace each placeholder in the precondition with the corresponding grounded argument
    for (const string& precondition_arg : precondition.get_args())
    {
      auto it_action_arg = find(action_args.begin(), action_args.end(), precondition_arg);

      if (it_action_arg != action_args.end())
      {
        int index = distance(action_args.begin(), it_action_arg);
        auto it_ground = next(grounded_args.begin(), index);

        if (it_ground != grounded_args.end())
        {
          replaced_args.push_back(*it_ground);  // Replace placeholder
        }
        else
        {
          cerr << "Error: Grounded argument not found for placeholder: " << precondition_arg << endl;
          return false;
        }
      }
      else
      {
        // If the precondition_arg is not found in action_args, it might be a concrete value
        replaced_args.push_back(precondition_arg);  // Use the argument as it is
      }
    }

    // Create a grounded condition for the precondition and check if it's met in the current state
    GroundedCondition grounded_precondition(precondition.get_predicate(), replaced_args, precondition.get_truth());
    if (current_state.find(grounded_precondition) == current_state.end())
    {
      return false;  // Precondition not met
    }
  }

  return true;  // All preconditions are met
}

list<GroundedCondition> ground_action_effects(const Action& action, const list<string>& grounded_args)
{
  list<GroundedCondition> grounded_effects;
  const list<string>& action_args = action.get_args();  // Placeholders

  // Iterate over each effect of the action
  for (const Condition& effect : action.get_effects())
  {
    list<string> replaced_args;  // List to store replaced arguments

    // Replace each placeholder in the effect with the corresponding grounded argument
    for (const string& effect_arg : effect.get_args())
    {
      auto it_action_arg = find(action_args.begin(), action_args.end(), effect_arg);

      if (it_action_arg != action_args.end())
      {
        int index = distance(action_args.begin(), it_action_arg);
        auto it_ground = next(grounded_args.begin(), index);

        if (it_ground != grounded_args.end())
        {
          replaced_args.push_back(*it_ground);  // Replace placeholder
        }
        else
        {
          cerr << "Error: Grounded argument not found for placeholder: " << effect_arg << endl;
        }
      }
      else
      {
        // This might indicate that the effect_arg is not a placeholder but a concrete value.
        replaced_args.push_back(effect_arg);  // Use the argument as it is
      }
    }

    // Create a grounded condition for the effect (including its truth value) and add it to the list
    GroundedCondition grounded_effect(effect.get_predicate(), replaced_args, effect.get_truth());
    grounded_effects.push_back(grounded_effect);
  }

  return grounded_effects;
}

list<GroundedAction> planner(Env* env)
{
  // Create an open list (priority queue) for states to explore
  std::priority_queue<Node, std::vector<Node>, NodeCostComparator> open_list;
  // Create a closed list (unordered set) for states already explored
  std::unordered_set<Node, NodeHasher, NodeStateComparator> closed_list;
  // Create a set to store all open nodes
  std::unordered_set<Node, NodeHasher, NodeStateComparator> all_open_nodes;

  // get min_literals_added by iterating over all the actions and finding the minimum number of literals added by any
  // action
  min_literals_added = get_min_literals_added(env);
  if (min_literals_added == 0)
    min_literals_added = 1;
  cout << "Min literals added by any action: " << min_literals_added << endl;

  // Create a node for the initial state
  Node start_node(GroundedAction("", list<string>()), env->initial_conditions, nullptr, 0,
                  compute_heuristic(env->initial_conditions, env->goal_conditions));

  // Add the initial state to the open list
  open_list.push(start_node);
  all_open_nodes.insert(start_node);
  closed_list.insert(start_node);

  using namespace std::chrono;
  auto start_time = high_resolution_clock::now();
  int expanded_states = 0;

  // While the open list is not empty
  while (!open_list.empty())
  {
    // Pop the node with the lowest cost
    Node current_node = open_list.top();
    open_list.pop();
    closed_list.insert(current_node);
    all_open_nodes.insert(current_node);
    expanded_states++;

    // The goal condition can exist as a subset of the current state
    bool goal_found = true;
    for (GroundedCondition gc : env->goal_conditions)
    {
      if (current_node.state.find(gc) == current_node.state.end())
      {
        goal_found = false;
        break;
      }
    }
    if (goal_found)
    {
      cout << "Goal found!" << endl;
      // Return the path to the goal
      list<GroundedAction> plan;
      // list of all the states in the path
      shared_ptr<Node> current = make_shared<Node>(current_node);
      while (current->parent != nullptr)
      {
        plan.push_front(current->action);
        current = current->parent;
      }
      auto end_time = high_resolution_clock::now();
      auto duration = duration_cast<milliseconds>(end_time - start_time).count();
      cout << "Time taken: " << duration << " ms" << endl;
      cout << "Number of States Expanded: " << expanded_states << endl;
      cout << "Number of Actions in the Plan: " << plan.size() << endl;
      return plan;
    }

    // For each action in the environment
    for (Action action : env->actions)
    {
      // print action
      // cout << action << endl;
      // Generate all combinations of arguments
      auto symbols = env->get_symbols();
      list<list<string>> combinations =
          generate_permutations(symbols, action.get_args().size());  // TODO: this doesnt have to be done every time,
                                                                     // can be done once and stored in a map

      for (const list<string>& args : combinations)
      {
        GroundedAction grounded_action(action.get_name(), args);
        if (are_all_preconditions_met(action, args, current_node.state))
        {
          // Apply the action to the current state // TODO: make this a function
          list<GroundedCondition> grounded_effects = ground_action_effects(action, args);
          // Apply these grounded effects to the current state to generate a new state
          unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_state =
              current_node.state;
          for (const GroundedCondition& effect : grounded_effects)
          {
            if (effect.get_truth())
            {
              new_state.insert(effect);  // Add positive effect
            }
            else
            {
              // For negated effect, find and remove the corresponding positive condition
              for (auto it = new_state.begin(); it != new_state.end();)
              {
                if (it->get_predicate() == effect.get_predicate() && it->get_arg_values() == effect.get_arg_values())
                {
                  it = new_state.erase(it);  // Remove the condition
                }
                else
                {
                  ++it;
                }
              }
            }
          }
          // Create a new node for the action
          Node new_node(grounded_action, new_state, make_shared<Node>(current_node), current_node.g_cost + 1,
                        compute_heuristic(new_state, env->goal_conditions));

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
      }
    }
  }
  auto end_time = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(end_time - start_time).count();
  cout << "Time taken: " << duration << " ms" << endl;
  cout << "Number of States Expanded: " << expanded_states << endl;
  // cout << "Number of Actions in the Plan: " << plan.size() << endl;
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