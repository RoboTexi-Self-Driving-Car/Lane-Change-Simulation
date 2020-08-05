//
//  simulation.h
//  CarGame
//
//  Created by HJKBD on 8/21/16.
//  Copyright © 2016 HJKBD. All rights reserved.
//

#ifndef MODEL_H
#define MODEL_H

#include "KdTree.hpp"
#include "layout.h"
#include "vec2D.h"
#include "inference.h"
#include "car.h"

using std::string;
using std::vector;

// Forward declaration
class Car;
class Simulation;
namespace Inference {
  class JointParticles;
  class MarginalInference;
}

//************************************************************************
// class Line
//************************************************************************

struct Line {
  int x1, y1, x2, y2;

  Line(float _x1, float _y1, float _x2, float _y2)
      : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}

  Line(vector<int>& row) {
    x1 = row[0] * (Globals::constant.BLOCK_TILE_SIZE);
    y1 = row[1] * (Globals::constant.BLOCK_TILE_SIZE);
    x2 = row[2] * (Globals::constant.BLOCK_TILE_SIZE);
    y2 = row[3] * (Globals::constant.BLOCK_TILE_SIZE);
  }

  Vector2f getstart() { return Vector2f(x1, y1); }

  Vector2f getend() { return Vector2f(x2, y2); }
};

//************************************************************************
// class Block
//************************************************************************

class Block {
public:
  Block() : startx(0), starty(0), endx(0), endy(0) {}

  Block(vector<int>& blockdata) {
    assert(blockdata.size() == 4);
    int unit = Globals::constant.BLOCK_TILE_SIZE;
    startx = blockdata[0] * unit;
    starty = blockdata[1] * unit;
    endx = blockdata[2] * unit;
    endy = blockdata[3] * unit;
    centerX = (startx + endx) / 2.0;
    centerY = (starty + endy) / 2.0;
  }

  Vector2f getCenter() const { return Vector2f(centerX, centerY); }

  int getWidth() const { return abs(endx - startx); }

  int getHeight() const { return abs(endy - starty); }

  bool containsPoint(int x, int y) const {
    if (x < startx) return false;
    if (y < starty) return false;
    if (x > endx) return false;
    if (y > endy) return false;
    return true;
  }

  // larger one
  bool containsPointLarger(int x, int y) const {
    int size = 2;
    int startx1 = startx - size;
    int starty1 = starty - size;
    int endx1 = endx + size;
    int endy1 = endy + size;
    if (x < startx1) return false;
    if (y < starty1) return false;
    if (x > endx1) return false;
    if (y > endy1) return false;
    return true;
  }

private:
  int startx, starty;
  int endx, endy;
  float centerX, centerY;
};

//************************************************************************
// class Simulation
//************************************************************************

class Simulation {
public:
  Simulation(Layout&);

  Simulation(const Simulation&);

  ~Simulation();

  // get the properties for the simulation class
  int getWidth() const { return layout.getWidth(); }

  int getHeight() const { return layout.getHeight(); }

  int getBeliefRows() const { return layout.getBeliefRows(); }

  int getBeliefCols() const { return layout.getBeliefCols(); }

  const vector<Block*>& getBlocks() const { return blocks; }

  const vector<Line*>& getLine() const { return lines; }

  const vector<Car*>& getCars() const { return cars; }

  const vector<Car*>& getOtherCars() const { return other_cars; }

  vector<Vector2f> getIntersectionCenter();

  vector<Block*>& getIntersectionGraph() { return interSections; }

  vector<Block*>& getAgentGraph() { return agentGraph; }

  vector<Block*>& getHostGraph() { return hostGraph; }

  vector<Block*>& getAllGraph() { return allGraph; }

  Block* getIntersection(float x, float y) const;

  Block& getGoal() const { return *goal; }

  Car* getHost() const { return host; }

  void setHost(Car* car);

  // utility function to help check simulation
  bool checkVictory() const;

  bool checkCollision(Car* car) const;

  bool inBounds(float x, float y) const;

  bool inBoundsLarger(float x, float y) const;

  bool inIntersection(float x, float y) const;

  int toindex(const Car* car) const { return car2index.at((size_t)car); }

private:
  Layout& layout;
  Block* goal;
  Car* host;
  // vector<vector<int>> othercardata;
  vector<Block*> blocks;
  vector<Line*> lines;
  vector<Car*> cars;
  vector<Car*> other_cars;
  vector<Block*> interSections;
  vector<Block*> agentGraph;
  vector<Block*> hostGraph;
  vector<Block*> allGraph;
  UMAP<size_t, int> car2index;

  void clearBlocks(vector<Block*>& blocks);

  void initBlocks();

  void initLines();

  void initIntersections();

  void initGraphs();

  // void initOtherCars();

  // void getStartNode();
};

#endif /* MODEL_H */
