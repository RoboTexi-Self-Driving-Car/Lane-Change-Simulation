#ifndef CAR_H
#define CAR_H

#include "vec2D.h"

//abstract class
class Car {
private:
  Vector2f pos;
  Vector2f velocity;
  Vector2f dir;

public:
  float wheelAngle;
  float maxSpeed;
  float friction;
  float maxWheelAngle;
  float maxaccler;
  float minSpeed;

public:
  void init(const string & dir);
  virtual void init();
  constexpr const static float LENGTH = 25.0;
  constexpr const  static float WIDTH = 12.5;
  const static float RADIUS;

  Car(){}

  Car(const Vector2f& _pos, string&& dir, const Vector2f& _velocity): pos(_pos), velocity(_velocity) { init(dir); }

  Car(const Vector2f& _pos, const string& dir, const Vector2f& _velocity): pos(_pos), velocity(_velocity) { init(dir); }

  Car(const Vector2f& _pos, const string& dir, Vector2f&& _velocity): pos(_pos), velocity(_velocity) { init(dir); }

  Car(const Vector2f& _pos, string&& dir, Vector2f&& _velocity): pos(_pos), velocity(_velocity) { init(dir); }
  Car(const Vector2f& _pos, const Vector2f& dir_, const Vector2f& _velocity):
    pos(_pos), dir(dir_), velocity(_velocity) { init(); }

  virtual ~Car() {};
  virtual bool isHost() { return false; }
  virtual void autonomousAction(const vector<Vector2f>&, const Model&, kdtree::kdtree<point<float>>* tree=NULL) {};
  virtual void autonomousAction2(const vector<Vector2f>&, const Model&, int i = 1) {};
  virtual void setup();

  Vector2f getPos() const  { return pos; }

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

  vector<Vector2f> getBounds(Car&car, float LEN, float WID);

  //# http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
  bool collides(const Vector2f& otherPos, const vector<Vector2f>& otherBounds);
  //carfufl not to too use the function, this is used for planning ahead
  void setVelocity(float amount);
  bool carInIntersection(const Model& state);
  bool isCloseToOtherCar(const Model& model) const;

};


//derived class
class Host: public Car {
  //car model
private:
  int nodeId;
  int pre;
public:

  Host():Car(), nodeId{0}, pre{-1} {}
  Host(const Vector2f& _pos, string&& _dir, const Vector2f& _velocity): Car(_pos, _dir, _velocity), nodeId(0),pre(-1) { setup(); }
  Host(const Vector2f& _pos, const string& _dir, const Vector2f& _velocity): Car(_pos, _dir, _velocity),nodeId(0),pre(-1) { setup(); }
  Host(const Vector2f& _pos, const string& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity),nodeId(0),pre(-1) { setup(); }
  Host(const Vector2f& _pos, string&& _dir, Vector2f&& _velocity): Car(_pos, _dir, _velocity),nodeId(0), pre(-1) { setup(); }
  Host(const Car& car): Car(car.getPos(), car.getDir(),car.getVelocity()),nodeId(0), pre(-1) { setup(); }
  ~Host() {};

  virtual void setup();
  bool isHost() { return true; }
  void autonomousAction(const vector<Vector2f>&path, const Model& model, kdtree::kdtree<point<float>>* tree);
  void autonomousAction2(const vector<Vector2f>&path, const Model& model, int i = 1);
  UMAP<string, float>  getAutonomousActions(const vector<Vector2f>& path, const Model& model, kdtree::kdtree<point<float>>* tree);
  UMAP<string, float>  getAutonomousActions2(const vector<Vector2f>& path, const Model& model);
  void makeObse(const Model& state);
};

#endif // CAR_H