#ifndef DECISION_MAKING_H
#define DECISION_MAKING_H

#include "globals.h"
#include "model.h"

static float manhaDistance(const vec2f& position, const vec2f& goal) {
  vec2f xy1 = position;
  vec2f xy2 = goal;
  return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1]);
}

static Vector2f setToOne(Vector2f& vec) {
  float length = vec.Length();
  if (length == 0) return vec;
  if (vec[0] != 0) return vec / vec[0];
  return vec / vec[1];
}

static const unordered_map<std::string, pii> directions = {
    {"east", {1, 0}},        {"northeast", {1, -1}}, {"north", {0, -1}},
    {"northwest", {-1, -1}}, {"west", {-1, 0}},      {"southwest", {-1, 1}},
    {"south", {0, 1}},       {"southeast", {1, 1}},
};

static inline int xToCol(float x) {
  return int(x / (Globals::constant.BELIEF_TILE_SIZE));
}

static inline int yToRow(float y) {
  return int((y / Globals::constant.BELIEF_TILE_SIZE));
}

float rowToY(int row) {
  return (row + 0.5) * Globals::constant.BELIEF_TILE_SIZE;
}

float colToX(int col) {
  return (col + 0.5) * Globals::constant.BELIEF_TILE_SIZE;
}

static vec2f corToCenter(const vec2f& pos) {
  float x = pos[0];
  float y = pos[1];
  int col = xToCol(x);
  int row = yToRow(y);
  float newx = colToX(col);
  float newy = rowToY(row);
  return vec2f(newx, newy);
}

class DecisionAgent {
  /*
  This class provides some common elements to all of your
  multi-agent searchers.  Any methods defined here will be available
  to the  ExpectimaxCarAgent.
  */
  /*
  Note: this is an abstract class: one that should not be instantiated.  It's
  only partially specified, and designed to be extended.  Agent (game.py)
  is another abstract class.
  """
  */
private:
  int depth;
  unsigned int index;

public:
  static const vector<std::string> hostActions;
  static const vector<std::string> otherActions;
  static const unordered_map<std::string, float> actionReward;
  // static const unorder_mapd<std::string, float> command;
  DecisionAgent(int dep = 2, int ind = 0) : depth(dep), index(ind) {}
  std::unordered_map<std::string, vec2f> generateLegalActions(const Model&,
                                                              int);
  Model generateSuccessor(const Model&, int,
                          const std::pair<std::string, vec2f>&);
  void ApplyAction(const Model&, int, const std::string&);

  float evaluationFunction(const Model&);
  float value(const Model&, int, int);
  float maxvalue(const Model&, int, int);
  float expecvalue(const Model&, int, int);
  std::pair<std::string, vec2f> getAction(const Model& model);
  bool isCloseToOtherCar(Car* car, const Model& model) const;
};

#endif /* DECISION_MAKING_H */
