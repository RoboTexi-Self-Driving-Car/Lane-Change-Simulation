#ifndef DECISION_MAKING_2_H
#define DECISION_MAKING_2_H

#include "search.h"
#include "search2.h"

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

/*
 * This class provides some common elements to all of your
 * multi-agent searchers.  Any methods defined here will be available
 * Note: this is an abstract class: one that should not be instantiated.  It's
 * only partially specified, and designed to be extended.  Agent (game.py)
 * is another abstract class.
 */
class DecisionAgent2 {
public:
  static const vector<std::string> hostActions;
  static const vector<std::string> otherActions;
  static const unordered_map<std::string, float> actionReward;
  // static const unorder_mapd<std::string, float> command;

  DecisionAgent2(int dep = 2, int ind = 0) : depth(dep), index(ind) {}

  vector<string> generateLegalActions(const Model&);

  const vector<vector<vec2f>>& generatePaths(const Model&, vector<string>&);

  const vector<vector<vec2f>>& generatePaths2(const Model& mod,
                                              vector<string>&);

  void ApplyAction(const Model&, int, const std::string&);

  float evaluationPath(const Model&, const vector<vec2f>& path,
                       vector<int>& carintentions);

  bool getPath(const Model& model, vector<vec2f>& mypath,
               vector<int>& carIntentions);

  bool getPath2(const Model& model, vector<vec2f>& mypath,
                vector<int>& carIntentions);

  const vector<vector<vec2f>> getPaths() { return paths; }

  bool isCloseToOtherCar(Car* car, const Model& model) const;

  bool isChangeRequired(Car* mycar, const Model& model);

private:
  int depth;
  unsigned int index;
  vector<vector<vec2f>> paths;
};

#endif /* DECISION_MAKING_2_H */
