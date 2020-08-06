#ifndef CAR_H
#define CAR_H

#include "inference.h"
#include "simulation.h"

class Actor;
class Simulation;
namespace Inference {
  class JointParticles;
  class MarginalInference;
}

/*
 * Actor object initilization now
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

// abstract class
class Actor {
public:
  float wheel_angle;
  float max_speed;
  float friction;
  float max_wheel_angle;
  float max_accler;
  float min_speed;

  constexpr const static float LENGTH = 25.0;
  constexpr const static float WIDTH = 12.5;
  const static float RADIUS;

public:
  Actor() {}

  Actor(const Vector2f& _pos, string&& dir, const Vector2f& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Actor(const Vector2f& _pos, const string& dir, const Vector2f& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Actor(const Vector2f& _pos, const string& dir, Vector2f&& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }

  Actor(const Vector2f& _pos, string&& dir, Vector2f&& _velocity)
      : pos(_pos), velocity(_velocity) {
    init(dir);
  }
  Actor(const Vector2f& _pos, const Vector2f& dir_, const Vector2f& _velocity)
      : pos(_pos), dir(dir_), velocity(_velocity) {
    init();
  }

  virtual ~Actor(){};

  void init(const string& dir);

  virtual void init();

  virtual bool isHost() { return false; }

  virtual void autonomousAction(const vector<Vector2f>&, const Simulation&, kdtree::kdtree<point<float>>* tree = nullptr) {};

  virtual void autonomousAction(const vector<Vector2f>&, const Simulation&, int i = 1) {};

  virtual void setup();

  Vector2f getPos() const { return pos; }

  void setPos(const Vector2f& pos) { this->pos = pos; }

  Vector2f getDir() const { return dir; }

  Vector2f getVelocity() const { return velocity; }

  void turnCarTowardsWheels();

  void update();

  void decellerate(float amount);

  void turnWheelsTowardsStraight() { wheel_angle = 0.0; }

  void applyFriction() { decellerate(friction); }

  void setWheelAngle(float angle);

  void accelerate(float amount);

  vector<Vector2f> getBounds();

  vector<Vector2f> getBounds(Actor& car, float LEN, float WID);

  // http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
  bool collides(const Vector2f& otherPos, const vector<Vector2f>& otherBounds);

  // carfufl not to too use the function, this is used for planning ahead
  void setVelocity(float amount);

  bool carInIntersection(const Simulation& simulation);

  bool isCloseToOtherCar(const Simulation& simulation) const;

private:
  Vector2f pos;
  Vector2f velocity;
  Vector2f dir;
};

/*
 * derived class
 */
class Host : public Actor {
public:
  Host() : Actor(), node_id{0}, pre{-1} {}

  Host(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity)
      : Actor(_pos, _dir, _velocity), node_id(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity)
      : Actor(_pos, _dir, _velocity), node_id(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity)
      : Actor(_pos, _dir, _velocity), node_id(0), pre(-1) {
    setup();
  }

  Host(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity)
      : Actor(_pos, _dir, _velocity), node_id(0), pre(-1) {
    setup();
  }

  Host(const Actor& car)
      : Actor(car.getPos(), car.getDir(), car.getVelocity()), node_id(0), pre(-1) {
    setup();
  }

  ~Host(){};

  virtual void setup();

  bool isHost() { return true; }

  void autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, kdtree::kdtree<point<float>>* tree);

  void autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, int intention = 1);

  UMAP<string, float> getAutonomousActions(const vector<Vector2f>& path,
                                           const Simulation& simulation,
                                           kdtree::kdtree<point<float>>* tree);

  UMAP<string, float> getAutonomousActions(const vector<Vector2f>& path,
                                           const Simulation& simulation);

  void makeObservation(const Simulation& simulation);

private:
  int node_id;
  int pre;
};

// derived class for other cars
class Car : public Actor {
public:
  unsigned int timer = 0;
  bool stop_flag = false;
  std::queue<float> history;
  bool has_inference;
  Inference::MarginalInference* inference;

public:
  Car() : Actor() {}

  Car(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity)
      : Actor(_pos, _dir, _velocity) {
    setup();
  }

  Car(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity)
      : Actor(_pos, _dir, _velocity) {
    setup();
  }

  Car(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity)
      : Actor(_pos, _dir, _velocity) {
    setup();
  }

  Car(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity)
      : Actor(_pos, _dir, _velocity) {
    setup();
  }

  Car(const Actor& car) : Actor(car.getPos(), car.getDir(), car.getVelocity()) {}

  ~Car() {
    if (inference != nullptr) delete inference;
  };

  virtual void setup();

  bool isHost() { return false; }

  std::queue<float>& getHistory() { return history; }

  void autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, kdtree::kdtree<point<float>>* tree);

  void autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, int intention = 1);

  Inference::MarginalInference* getInference(int index, const Simulation& simulation);

  Vector2f getObservation() { return getVelocity(); }
};

#endif /* CAR_H */
