#ifndef LAYOUT_H
#define LAYOUT_H

#include <fstream>

#include "globals.h"
#include "thirdparty/picojson.h"

class Layout {
public:
  // constructor for layout
  Layout(std::string worldname);

  ~Layout() {}

  int getWidth();

  int getHeight();

  int getStartX();

  int getStartY();

  int getBeliefRows();

  int getBeliefCols();

  vector<vector<int>> getLineData();

  vector<vector<int>> getOtherData();

  std::string getHostDir();

  vector<int> getGoal();

  vector<vector<int>> getBlockData();

  vector<vector<int>> getIntersectionData();

  vector<vector<int>> getAgentGraph();

  vector<vector<int>> getHostGraph();

private:
  void loadData(std::string filename);

  void assertValid();

  picojson::value data;
};

#endif /* LAYOUT_H */
