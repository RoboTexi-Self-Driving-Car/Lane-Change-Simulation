//
//  search.h
//  CarGame
//
//  Created by HJKBD on 8/22/16.
//  Copyright © 2016 HJKBD. All rights reserved.
//

/*
 * read first
 * cost function defined in both the generate successsor and also the evaluation
 * function depends on the applications, the cost function is adjusted to suit the
 * best
 */
#ifndef SEARCH_H
#define SEARCH_H

#include <cmath>
#include <iostream>
#include <queue>

#include "model.h"
#include "vec2D.h"

using std::ostream;
using std::vector;
using std::priority_queue;

typedef Vector2d<float> vec2f;
typedef std::pair<vec2f, vec2f> pvff;
typedef std::pair<int, int> pii;

namespace SEARCH {

struct State {
  pvff current;
  list<char> actions;
  float cost;
  float heu;

  State() : cost(0), heu(0) {};

  State(const pvff& p1, float c = 0, float h = 0)
      : current(p1), cost(c), heu(h) {};

  State(const pvff& p1, const list<char>& actions_, float c = 0, float h = 0)
      : current(p1), actions(actions_), cost(c), heu(h) {};

  State(const State& s)
      : current(s.current), actions(s.actions), cost(s.cost), heu(s.heu) {}

  State(State&& s)
      : current(std::move(s.current)),
        actions(std::move(s.actions)),
        cost(std::move(s.cost)),
        heu(std::move(s.heu)) {}

  State& operator=(const State& s);

  // State& operator=(State&& s);

  friend ostream& operator<<(ostream& os, const State& s);

  bool operator<(const State& s2) const {
    return (cost + heu) > (s2.cost + s2.heu);
  }
};

class Search {
public:
  Search(Model* m, const vec2f& goal);
  State search();
  vector<State> getSuccessors(const State& state);
  bool isGoal(State&);
  size_t num_action() { return angle.size(); }
  vector<vec2f>& path() {
    smooth();
    return pa;
  }
  float evaluation(const vec2f& position);
  void smooth();

private:
  Model* model;
  int unitdistanace;
  vec2f goal;
  State start;
  float cost;
  vector<float> angle;
  vector<vec2f> pa;
  // enum {left90, left45, strainght, right45, right90};
  // float angle[9] = {60, 45, 30, 15, 0, -15, -30, -45, -60};
  // enum {east, north, west, south};

  int xToCol(float x) { return int((x / unitdistanace)); }
  int yToRow(float y) { return int((y / unitdistanace)); }
  float rowToY(int row) { return (row + 0.5) * unitdistanace; }
  float colToX(int col) { return (col + 0.5) * unitdistanace; }
  vector<vec2f> path(list<char>&);

  /**
   * "The Manhattan distance heuristic for a PositionSearchProblem"
   */
  float manhattanHeuristic(const vec2f& position) {
    vec2f xy1 = position;
    vec2f xy2 = goal;
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1]);
  }
};

}  // namespace SEARCH

#endif /* search_h */
