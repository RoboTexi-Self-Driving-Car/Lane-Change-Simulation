#ifndef DECISION_MAKING_H
#define DECISION_MAKING_H

#include "search.h"

inline static int xToCol(float x) {
  return int(x / (Globals::constant.BELIEF_TILE_SIZE));
}

inline static int yToRow(float y) {
  return int((y / Globals::constant.BELIEF_TILE_SIZE));
}

inline static float rowToY(int row) {
  return (row + 0.5) * Globals::constant.BELIEF_TILE_SIZE;
}

inline static float colToX(int col) {
  return (col + 0.5) * Globals::constant.BELIEF_TILE_SIZE;
}

inline float manhattanDistance(const Vector2f& v1, const Vector2f& v2) {
  float distance = abs(v1[0] - v2[0]) + abs(v1[1] - v2[1]);
  return distance;
}

/*
 * This class provides some common elements to all of your
 * multi-agent searchers.  Any methods defined here will be available
 * Note: this is an abstract class: one that should not be instantiated.  It's
 * only partially specified, and designed to be extended.  Agent (game.py)
 * is another abstract class.
 */
class DecisionMaker {
public:
  static vector<std::string> hostActions;
  static vector<std::string> otherActions;
  static unordered_map<std::string, float> actionReward;
  // static const unorder_mapd<std::string, float> command;

  DecisionMaker(int dep = 2, int ind = 0) : depth(dep), index(ind) {}

  vector<string> generateLegalActions(const Simulation&);

  vector<vector<vec2f>>& generatePaths(const Simulation&, vector<string>&);

  void ApplyAction(const Simulation&, int, const std::string&);

  float evaluationPath(const Simulation&, const vector<vec2f>& path, vector<int>& car_intentions);

  bool getPath(const Simulation& simulation, vector<vec2f>& final_path, vector<int>& carIntentions);

  vector<vector<vec2f>> getPaths() { return paths; }

  bool isCloseToOtherCar(Car* car, const Simulation& simulation) const;

  bool isChangeRequired(Car* ego_car, const Simulation& simulation);

private:
  int depth;
  unsigned int index;
  vector<vector<vec2f>> paths;
};

#endif /* DECISION_MAKING_H */
