//
//  model.h
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
// #include "inference.h"

using std::string;
using std::vector;

// Forward declaration
class Car;
class Model;

/*********************************** Inference *******************************/
namespace Inference {

enum State {cooperative, aggressive};

const vector<string> intentions{"cooperative", "aggressive"};

const UMAP<string, int> Intention_To_Index{{"cooperative", 0}, {"aggressive", 1}};

//************************************************************************
// class JointParticles
//************************************************************************

class JointParticles {
public:
  JointParticles(int num = 600) : numParticles(num), numAgents(0){};

  void initializeUniformly(const Model& model,
                           const vector<string>& intentions);

  void initializeParticles();

  void observe(const Model& model);

  Counter<vector<string>> getBelief();

  vector<string> sample(Counter<vector<string>>& counter);

  pff getMeanStandard(queue<float>& history, const string& intention);

private:
  int numParticles;
  int numAgents;
  vector<string> legalIntentions;
  vector<Car*> agents;
  Counter<vector<string>> beliefs;
  vector<vector<string>> particles;
};

static JointParticles jointInference = JointParticles();

//************************************************************************
// class MarginalInference
//************************************************************************

class MarginalInference {
public:
  MarginalInference(int index, const Model& model);
  void initializeUniformly(const Model& gameState);
  void observe(const Model& gameState);
  vector<float> getBelief();

private:
  vector<string> legalIntentions;
  int index;
};

}  // namespace Inference
/******************************************************************************/

inline float manhattanDistance(const Vector2f& v1, const Vector2f& v2) {
  float distance = abs(v1[0] - v2[0]) + abs(v1[1] - v2[1]);
  return distance;
}

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
// class Model
//************************************************************************

class Model {
public:
  Model(Layout&);

  Model(const Model&);

  ~Model();

  // get the properties for the model class
  int getWidth() const { return layout.getWidth(); }

  int getHeight() const { return layout.getHeight(); }

  int getBeliefRows() const { return layout.getBeliefRows(); }

  int getBeliefCols() const { return layout.getBeliefCols(); }

  const vector<Block*>& getBlocks() const { return blocks; }

  const vector<Line*>& getLine() const { return lines; }

  const vector<Car*>& getCars() const { return cars; }

  const vector<Car*>& getOtherCars() const { return otherCars; }

  vector<Vector2f> getIntersectionCenter();

  vector<Block*>& getIntersectionGraph() { return interSections; }

  vector<Block*>& getAgentGraph() { return agentGraph; }

  vector<Block*>& getHostGraph() { return hostGraph; }

  vector<Block*>& getAllGraph() { return allGraph; }

  Block* getIntersection(float x, float y) const;

  Block& getFinish() const { return *finish; }

  Car* getHost() const { return host; }

  void setHost(Car* car);

  // utility function to help check model
  bool checkVictory() const;

  bool checkCollision(Car* car) const;

  bool inBounds(float x, float y) const;

  bool inBoundsLarger(float x, float y) const;

  bool inIntersection(float x, float y) const;

  int toindex(const Car* car) const { return cartoindex.at((size_t)car); }

private:
  Layout& layout;
  Block* finish;
  Car* host;
  // vector<vector<int>> othercardata;
  vector<Block*> blocks;
  vector<Line*> lines;
  vector<Car*> cars;
  vector<Car*> otherCars;
  vector<Block*> interSections;
  vector<Block*> agentGraph;
  vector<Block*> hostGraph;
  vector<Block*> allGraph;
  UMAP<size_t, int> cartoindex;

  void clearBlocks(vector<Block*>& blocks);

  void initBlocks();

  void initLines();

  void initIntersections();

  void initGraphs();

  // void initOtherCars();

  // void getStartNode();
};

/*
 * Car object initilization now
 */

static UMAP<string, pff> direction = {
  {"east", {1, 0}},
  {"west", {-1, 0}},
  {"south", {1, 0}},
  {"north", {-1, 0}},
  {"northeast", {1, 1}},
  {"northwest", {-1, 1}},
  {"southeast", {1, -1}},
  {"southwest", {-1, -1}}
};

//************************************************************************
// class Car: Abstract class
//************************************************************************

class Car {
public:
  float wheelAngle;
  float maxSpeed;
  float friction;
  float maxWheelAngle;
  float maxaccler;
  float minSpeed;

  constexpr const static float LENGTH = 25.0;
  constexpr const static float WIDTH = 12.5;
  const static float RADIUS;

public:
  Car() {}

  Car(const Vector2f& _pos, string&& dir, const Vector2f& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Car(const Vector2f& _pos, const string& dir, const Vector2f& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Car(const Vector2f& _pos, const string& dir, Vector2f&& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Car(const Vector2f& _pos, string&& dir, Vector2f&& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }
  Car(const Vector2f& _pos, const Vector2f& dir_, const Vector2f& _velocity)
      : pos(_pos), dir(dir_), velocity(_velocity) {
    init();
  }

  virtual ~Car(){};

  void init(const string& dir);

  virtual void init();

  virtual bool isHost() { return false; }

  virtual void autonomousAction(const vector<Vector2f>&, const Model&,
                                kdtree::kdtree<point<float>>* tree = NULL){};

  virtual void autonomousAction2(const vector<Vector2f>&, const Model&,
                                 int i = 1){};

  virtual void setup();

  Vector2f getPos() const { return pos; }

  void setPos(const Vector2f& pos) { this->pos = pos; }

  Vector2f getDir() const { return dir; }

  Vector2f getVelocity() const { return velocity; }

  void turnCarTowardsWheels();

  void update();

  void decellerate(float amount);

  void turnWheelsTowardsStraight() { wheelAngle = 0.0; }

  void applyFriction() { decellerate(friction); }

  void setWheelAngle(float angle);

  void accelerate(float amount);

  vector<Vector2f> getBounds();

  vector<Vector2f> getBounds(Car& car, float LEN, float WID);

  //#
  //http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
  bool collides(const Vector2f& otherPos, const vector<Vector2f>& otherBounds);

  // carfufl not to too use the function, this is used for planning ahead
  void setVelocity(float amount);

  bool carInintersection(const Model& state);

  bool isCloseToOtherCar(const Model& model) const;

private:
  Vector2f pos;
  Vector2f velocity;
  Vector2f dir;
};

//************************************************************************
// class Host
//************************************************************************

class Host : public Car {
public:
  Host() : Car(), nodeId{0}, pre{-1} {}

  Host(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity)
      : Car(_pos, _dir, _velocity), nodeId(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity)
      : Car(_pos, _dir, _velocity), nodeId(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity)
      : Car(_pos, _dir, _velocity), nodeId(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity)
      : Car(_pos, _dir, _velocity), nodeId(0), pre(-1) {
    setup();
  }

  Host(const Car& car)
      : Car(car.getPos(), car.getDir(), car.getVelocity()), nodeId(0), pre(-1) {
    setup();
  }

  ~Host(){};

  virtual void setup();

  bool isHost() { return true; }

  void autonomousAction(const vector<Vector2f>& path, const Model& model,
                        kdtree::kdtree<point<float>>* tree);

  void autonomousAction2(const vector<Vector2f>& path, const Model& model,
                         int i = 1);

  UMAP<string, float> getAutonomousActions(const vector<Vector2f>& path,
                                           const Model& model,
                                           kdtree::kdtree<point<float>>* tree);

  UMAP<string, float> getAutonomousActions2(const vector<Vector2f>& path,
                                            const Model& model);

  void makeObse(const Model& state);

private:
  int nodeId;
  int pre;
};

//************************************************************************
// class Agent
//************************************************************************

// derived class for other cars
class Agent : public Car {
public:
  unsigned int timer = 0;
  bool stopflag = false;
  std::queue<float> history;
  bool hasinference;
  Inference::MarginalInference* inference;

public:
  Agent() : Car() {}

  Agent(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity)
      : Car(_pos, _dir, _velocity) {
    setup();
  }

  Agent(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity)
      : Car(_pos, _dir, _velocity) {
    setup();
  }

  Agent(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity)
      : Car(_pos, _dir, _velocity) {
    setup();
  }

  Agent(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity)
      : Car(_pos, _dir, _velocity) {
    setup();
  }

  Agent(const Car& car) : Car(car.getPos(), car.getDir(), car.getVelocity()) {}

  ~Agent() {
    if (inference != NULL) delete inference;
  };

  virtual void setup();

  bool isHost() { return false; }

  std::queue<float>& getHistory() { return history; }

  void autonomousAction(const vector<Vector2f>& vec2, const Model& model,
                        kdtree::kdtree<point<float>>* tree);

  void autonomousAction2(const vector<Vector2f>& path, const Model& model,
                         int i = 1);

  Inference::MarginalInference* getInference(int index, const Model& model);

  Vector2f getObserv() { return getVelocity(); }
};

#endif /* MODEL_H */
