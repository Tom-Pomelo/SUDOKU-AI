#include <climits>
#include <unordered_map>
#include <unordered_set>
#include <iostream> // TODO: debug

#include "BTSolver.hpp"

using namespace std;

// =====================================================================
// Constructors
// =====================================================================

BTSolver::BTSolver(SudokuBoard input, Trail* _trail, string val_sh,
                   string var_sh, string cc)
    : sudokuGrid(input.get_p(), input.get_q(), input.get_board()),
      network(input) {
  valHeuristics = val_sh;
  varHeuristics = var_sh;
  cChecks = cc;

  trail = _trail;
}

// =====================================================================
// Consistency Checks
// =====================================================================

// Basic consistency check, no propagation done
bool BTSolver::assignmentsCheck(void) {
  for (Constraint c : network.getConstraints())
    if (!c.isConsistent()) return false;

  return true;
}

// =================================================================
// Arc Consistency
// =================================================================
bool BTSolver::arcConsistency(void) {
  vector<Variable*> toAssign;
  vector<Constraint*> RMC = network.getModifiedConstraints();
  for (int i = 0; i < RMC.size(); ++i) {
    vector<Variable*> LV = RMC[i]->vars;
    for (int j = 0; j < LV.size(); ++j) {
      if (LV[j]->isAssigned()) {
        vector<Variable*> Neighbors = network.getNeighborsOfVariable(LV[j]);
        int assignedValue = LV[j]->getAssignment();
        for (int k = 0; k < Neighbors.size(); ++k) {
          Domain D = Neighbors[k]->getDomain();
          if (D.contains(assignedValue)) {
            if (D.size() == 1) return false;
            if (D.size() == 2) toAssign.push_back(Neighbors[k]);
            trail->push(Neighbors[k]);
            Neighbors[k]->removeValueFromDomain(assignedValue);
          }
        }
      }
    }
  }
  if (!toAssign.empty()) {
    for (int i = 0; i < toAssign.size(); ++i) {
      Domain D = toAssign[i]->getDomain();
      vector<int> assign = D.getValues();
      trail->push(toAssign[i]);
      toAssign[i]->assignValue(assign[0]);
    }
    return arcConsistency();
  }
  return network.isConsistent();
}

/**
 * Part 1 Implement the Forward Checking Heuristic
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: a pair of a map and a bool. The map contains the pointers to all
 * MODIFIED variables, mapped to their MODIFIED domain. The bool is true if
 * assignment is consistent, false otherwise.
 */
pair<map<Variable*, Domain>, bool> BTSolver::forwardChecking(void) {
  map<Variable*, Domain> m;
  bool flag = true;
  for (Variable* v : network.getVariables()) {
    if (v->isAssigned()) {
      for (Variable* neighbor : network.getNeighborsOfVariable(v)) {
        if (neighbor->getDomain().contains(v->getAssignment())) {
          trail->push(neighbor);
          neighbor->removeValueFromDomain(v->getAssignment());
          m[neighbor] = neighbor->getDomain();
        }
      }
    }
  }
  for (Constraint* c : network.getModifiedConstraints()) {
    if (!c->isConsistent()) {
      flag = false;
      break;
    }
  }
  return make_pair(m, flag);
}

/**
 * Part 2 TODO: Implement both of Norvig's Heuristics
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * (2) If a constraint has only one possible place for a value
 *     then put the value there.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: a pair of a map and a bool. The map contains the pointers to all
 * variables that were assigned during the whole NorvigCheck propagation, and
 * mapped to the values that they were assigned. The bool is true if assignment
 * is consistent, false otherwise.
 */
pair<map<Variable*, int>, bool> BTSolver::norvigCheck(void) {
  // do FC
  for (Variable* v : network.getVariables()) {
    if (v->isAssigned()) {
      for (Variable* neighbor : network.getNeighborsOfVariable(v)) {
        if (neighbor->getDomain().contains(v->getAssignment())) {
          trail->push(neighbor);
          neighbor->removeValueFromDomain(v->getAssignment());
        }
      }
    }
  }

  // check if a value can be placed in a contraint
  map<Variable*, int> m;
  for (Constraint* c : network.getModifiedConstraints()) {
    // get all values and var
    vector<pair<int, vector<Variable*>>> store;
    for (Variable* var : c->vars) {
      for (int val : var->getValues()) {
        
      }
    }
    // for each value check if it can be assigned to a variable
    // for (const auto& p : store) {
    //   if (p.second == nullptr) continue;
    //   Variable* var = p.second;
    //   int val = p.first;
    //   // trail->push(var);
    //   var->assignValue(val);
    //   m[var] = val;
    // }
  }
  return make_pair(m, network.isConsistent());
}

/**
 * Optional TODO: Implement your own advanced Constraint Propagation
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
bool BTSolver::getTournCC(void) { return false; }

// =====================================================================
// Variable Selectors
// =====================================================================

// Basic variable selector, returns first unassigned variable
Variable* BTSolver::getfirstUnassignedVariable(void) {
  for (Variable* v : network.getVariables())
    if (!(v->isAssigned())) return v;

  // Everything is assigned
  return nullptr;
}

/**
 * Part 1 Implement the Minimum Remaining Value Heuristic
 *
 * Return: The unassigned variable with the smallest domain
 */

Variable* BTSolver::getMRV(void) {
  Variable* v = nullptr;
  int n_remaining_value = INT_MAX;
  for (auto node : network.getVariables()) {
    if (!node->isAssigned() && node->size() < n_remaining_value) {
      v = node;
      n_remaining_value = node->size();
    }
  }
  return v;
}

/**
 * Part 2 TODO: Implement the Minimum Remaining Value Heuristic
 *                with Degree Heuristic as a Tie Breaker
 *
 * Return: The unassigned variable with the smallest domain and affecting the
 * most unassigned neighbors. If there are multiple variables that have the same
 * smallest domain with the same number of unassigned neighbors, add them to the
 * vector of Variables. If there is only one variable, return the vector of size
 * 1 containing that variable.
 */
vector<Variable*> BTSolver::MRVwithTieBreaker(void) {
  unordered_map<Variable*, pair<int, int>> map;
  // get remain value of all variables
  for (auto v : network.getVariables()) {
    if (v->isAssigned()) continue;
    map[v].first = v->size();
  }
  // get degree of all variables
  for (auto c : network.getConstraints()) {
    int unassigned = 0;
    for (auto v : c.vars) if (!v->isAssigned()) unassigned++;
    for (auto v : c.vars) {
      if (v->isAssigned()) continue;
      map[v].second += unassigned - 1;
    }
  }
  // find the selected variables
  int min_rem_val = INT_MAX;
  int max_degree = 0;
  for (auto p : map) {
    int rem_val = p.second.first;
    int degree = p.second.second;
    if (rem_val < min_rem_val) {
      min_rem_val = rem_val;
      max_degree = degree;
    } else if (rem_val == min_rem_val) {
      if (degree > max_degree) max_degree = degree;
    }
  }
  vector<Variable*> vars;
  for (auto p : map) {
    Variable* v = p.first;
    int rem_val = p.second.first;
    int degree = p.second.second;
    if (rem_val == min_rem_val && degree == max_degree) vars.push_back(v);
  }
  if (vars.empty()) vars.push_back(nullptr);
  return vars;
}

/**
 * Optional TODO: Implement your own advanced Variable Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
Variable* BTSolver::getTournVar(void) { return nullptr; }

// =====================================================================
// Value Selectors
// =====================================================================

// Default Value Ordering
vector<int> BTSolver::getValuesInOrder(Variable* v) {
  vector<int> values = v->getDomain().getValues();
  sort(values.begin(), values.end());
  return values;
}

/**
 * Part 1 Implement the Least Constraining Value Heuristic
 *
 * The Least constraining value is the one that will knock the least
 * values out of it's neighbors domain.
 *
 * Return: A list of v's domain sorted by the LCV heuristic
 *         The LCV is first and the MCV is last
 */
vector<int> BTSolver::getValuesLCVOrder(Variable* v) {
  // preprocess, count value occurence in every neighbor's domain
  unordered_map<int, int> occ;
  for (Variable* neighbor : network.getNeighborsOfVariable(v)) {
    if (neighbor->isAssigned()) continue;
    for (int val : neighbor->getValues()) occ[val]++;
  }

  // put the count for each value of v into priority queue
  vector<pair<int, int>> temp;
  for (int val : v->getValues()) temp.push_back(make_pair(val, occ[val]));
  auto cmp = [](pair<int, int> left, pair<int, int> right) {
    return left.second != right.second ? left.second < right.second
                                       : left.first < right.first;
  };
  sort(temp.begin(), temp.end(), cmp);

  // get the values with the order of LCV to MCV
  vector<int> res;
  for (pair<int, int> p : temp) res.push_back(p.first);
  return res;
}

/**
 * Optional TODO: Implement your own advanced Value Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
vector<int> BTSolver::getTournVal(Variable* v) { return vector<int>(); }

// =====================================================================
// Engine Functions
// =====================================================================

int BTSolver::solve(float time_left) {
  if (time_left <= 60.0) return -1;
  double elapsed_time = 0.0;
  clock_t begin_clock = clock();

  if (hasSolution) return 0;

  // Variable Selection
  Variable* v = selectNextVariable();

  if (v == nullptr) {
    for (Variable* var : network.getVariables()) {
      // If all variables haven't been assigned
      if (!(var->isAssigned())) {
        return 0;
      }
    }

    // Success
    hasSolution = true;
    return 0;
  }

  // Attempt to assign a value
  for (int i : getNextValues(v)) {
    // Store place in trail and push variable's state on trail
    trail->placeTrailMarker();
    trail->push(v);

    // Assign the value
    v->assignValue(i);

    // Propagate constraints, check consistency, recurse
    if (checkConsistency()) {
      clock_t end_clock = clock();
      elapsed_time += (float)(end_clock - begin_clock) / CLOCKS_PER_SEC;
      double new_start_time = time_left - elapsed_time;
      int check_status = solve(new_start_time);
      if (check_status == -1) {
        return -1;
      }
    }

    // If this assignment succeeded, return
    if (hasSolution) return 0;

    // Otherwise backtrack
    trail->undo();
  }
  return 0;
}

bool BTSolver::checkConsistency(void) {
  if (cChecks == "forwardChecking") return forwardChecking().second;

  if (cChecks == "norvigCheck") return norvigCheck().second;

  if (cChecks == "tournCC") return getTournCC();

  return assignmentsCheck();
}

Variable* BTSolver::selectNextVariable(void) {
  if (varHeuristics == "MinimumRemainingValue") return getMRV();

  if (varHeuristics == "MRVwithTieBreaker") return MRVwithTieBreaker()[0];

  if (varHeuristics == "tournVar") return getTournVar();

  return getfirstUnassignedVariable();
}

vector<int> BTSolver::getNextValues(Variable* v) {
  if (valHeuristics == "LeastConstrainingValue") return getValuesLCVOrder(v);

  if (valHeuristics == "tournVal") return getTournVal(v);

  return getValuesInOrder(v);
}

bool BTSolver::haveSolution(void) { return hasSolution; }

SudokuBoard BTSolver::getSolution(void) {
  return network.toSudokuBoard(sudokuGrid.get_p(), sudokuGrid.get_q());
}

ConstraintNetwork BTSolver::getNetwork(void) { return network; }
