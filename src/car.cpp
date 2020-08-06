#include "car.h"

//************************************************************************
// class Actor: methods and implementation
//************************************************************************

const float Actor::RADIUS = sqrt(pow(Actor::LENGTH, 2) + pow(Actor::WIDTH, 2));

void Actor::init(const string& dir) {
  pii p = direction[dir];
  this->dir = Vector2f(p.first, p.second);
  wheel_angle = 0;
  setup();
}

void Actor::init() {
  wheel_angle = 0;
  setup();
}

void Actor::setup() {
  max_speed = 5.0;
  min_speed = 1.0;
  friction = 0.5;
  max_wheel_angle = 130.0;
  max_accler = 2.0;
}

void Actor::turnCarTowardsWheels() {
  if (velocity.Length() > 0.0) {
    velocity.rotate(wheel_angle);
    dir = Vector2f(velocity[0], velocity[1]);
    dir.normalized();
  }
}

void Actor::update() {
  turnCarTowardsWheels();
  pos += velocity;
  turnWheelsTowardsStraight();
  applyFriction();
}

void Actor::decellerate(float amount) {
  float speed = velocity.Length();

  if (speed < min_speed) {
    speed = min_speed;
    return;
  }

  Vector2f frictionVec = velocity.get_reflection();
  frictionVec.normalized();
  frictionVec *= amount;
  velocity += frictionVec;
  float angle = velocity.get_angle_between(frictionVec);

  if (abs(angle) < 180) velocity = Vector2f(0, 0);
}

void Actor::setWheelAngle(float angle) {
  wheel_angle = angle;

  if (wheel_angle <= -max_wheel_angle) wheel_angle = -max_wheel_angle;
  if (wheel_angle >= max_wheel_angle) wheel_angle = max_wheel_angle;
}

void Actor::accelerate(float amount) {
  amount = std::min(amount, max_accler);

  if (amount < 0) decellerate(amount);
  if (amount == 0) return;

  Vector2f acceleration = Vector2f(dir[0], dir[1]);
  acceleration.normalized();
  acceleration *= amount;
  velocity += acceleration;

  if (velocity.Length() >= max_speed) {
    velocity.normalized();
    velocity *= max_speed;
  }
}

vector<Vector2f> Actor::getBounds() {
  dir.normalized();
  Vector2f perp_dir = dir.perpendicular();

  vector<Vector2f> bounds;
  bounds.push_back(pos + dir * float(LENGTH / 2) + perp_dir * float(WIDTH / 2));
  bounds.push_back(pos + dir * float(LENGTH / 2) - perp_dir * float(WIDTH / 2));
  bounds.push_back(pos - dir * float(LENGTH / 2) + perp_dir * float(WIDTH / 2));
  bounds.push_back(pos - dir * float(LENGTH / 2) - perp_dir * float(WIDTH / 2));

  return bounds;
}

vector<Vector2f> Actor::getBounds(Actor& car, float LEN, float WID) {
  Vector2f normalDir = normalized(car.getDir());
  Vector2f perp_dir = normalDir.perpendicular();

  vector<Vector2f> bounds;
  bounds.push_back(pos + dir * float(LEN / 2) + perp_dir * float(WID / 2));
  bounds.push_back(pos + dir * float(LEN / 2) - perp_dir * float(WID / 2));
  bounds.push_back(pos - dir * float(LEN / 2) + perp_dir * float(WID / 2));
  bounds.push_back(pos - dir * float(LEN / 2) - perp_dir * float(WID / 2));

  return bounds;
}

// http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
bool Actor::collides(const Vector2f& otherPos,
                     const vector<Vector2f>& otherBounds) {
  Vector2f diff = otherPos - pos;
  float dist = diff.Length();
  if (dist > RADIUS * 2) return false;

  vector<Vector2f> bounds = getBounds();
  Vector2f vec1 = bounds[0] - bounds[1];
  Vector2f vec2 = otherBounds[0] - otherBounds[1];
  vector<Vector2f> axis = {vec1,
                           vec1.perpendicular(),
                           vec2,
                           vec2.perpendicular()
                          };

  for (const auto& vec : axis) {
    pff result = projectPoints(bounds, vec);
    float minA = result.first;
    float maxA = result.second;
    result = projectPoints(otherBounds, vec);
    float minB = result.first;
    float maxB = result.second;
    bool leftmostA = (minA <= minB) ? true : false;
    bool overlap = false;

    if (leftmostA && maxA >= minB) overlap = true;
    if (!leftmostA && maxB >= minA) overlap = true;
    if (!overlap) return false;
  }
  return true;
}

// carfufl not to too use the function, this is used for planning ahead
void Actor::setVelocity(float amount) {
  Vector2f ve = Vector2f(dir[0], dir[1]);
  ve.normalized();
  ve *= amount;
  velocity = ve;
}

// check car is in instersection
bool Actor::carInIntersection(const Simulation& simulation) {
  vector<Vector2f> bounds = getBounds();
  for (const auto& point : bounds) {
    if (simulation.inIntersection(point[0], point[1])) return true;
  }
  return false;
}

bool Actor::isCloseToOtherCar(const Simulation& simulation) const {
  // check the master car is close to others
  vector<Actor*> cars = simulation.getAllCars();
  if (cars.size() == 0) return false;
  const Actor* obstaclecar = nullptr;
  float distance = 9999999;
  for (const Actor* car : cars) {
    if (car == this) continue;
    float cardis = abs(car->getPos()[0] - getPos()[0]) +
                   abs(car->getPos()[1] - getPos()[1]);
    if (cardis < distance) {
      distance = cardis;
      obstaclecar = car;
    }
  }

  if (!obstaclecar) return false;

  Vector2f diff = obstaclecar->getPos() - this->getPos();
  float angdiff = -diff.get_angle_between(this->getDir());
  if (abs(angdiff) > 90) return false;
  // std::cout<<angdiff <<std::endl;

  if ((abs(obstaclecar->getPos()[0] - getPos()[0]) <
       Globals::constant.BELIEF_TILE_SIZE * 1.5) &&
      (abs(obstaclecar->getPos()[1] - getPos()[1]) < Actor::WIDTH / 2))
    return true;
  return false;
}

//************************************************************************
// class Host: methods and implementation
//************************************************************************

void Host::setup() {
  max_speed = 3.0;
  friction = 1;
  max_wheel_angle = 45;
  max_accler = 1.5;
  min_speed = 1;
}

void Host::autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, kdtree::kdtree<point<float>>* tree = nullptr) {
  if (path.size() == 0) return;

  Vector2f oldPos = getPos();
  Vector2f oldDir = getDir();
  // Vector2f oldVel = getVelocity();
  UMAP<string, float> actions = getAutonomousActions(path, simulation, tree);
  assert(getPos() == oldPos);
  assert(getDir() == oldDir);

  // assert (getVelocity() == oldVel);
  if (actions.count("DRIVE_FORWARD")) {
    float percent = actions["DRIVE_FORWARD"];
    int sign = 1;
    if (percent < 0) sign = -1;
    percent = abs(percent);
    percent = percent > 0.0 ? percent : 0.0;
    percent = percent < 1.0 ? percent : 1.0;
    percent *= sign;
    accelerate(max_wheel_angle * percent);
    if (actions.count("TURN_WHEEL")) {
      float turnAngle = actions["TURN_WHEEL"];
      setWheelAngle(turnAngle);
    }
  }
}

void Host::autonomousAction(const vector<Vector2f>& path, const Simulation& simulation, int i) {
  if (path.size() == 0) return;

  Vector2f oldPos = getPos();
  Vector2f oldDir = getDir();
  // Vector2f oldVel = getVelocity();

  UMAP<string, float> actions = getAutonomousActions(path, simulation);

  assert(getPos() == oldPos);
  assert(getDir() == oldDir);
  // assert (getVelocity() == oldVel);

  if (actions.count("DRIVE_FORWARD")) {
    float percent = actions["DRIVE_FORWARD"];
    int sign = 1;
    if (percent < 0) sign = -1;
    percent = abs(percent);
    percent = percent > 0.0 ? percent : 0.0;
    percent = percent < 1.0 ? percent : 1.0;
    percent *= sign;
    accelerate(max_wheel_angle * percent);
    if (actions.count("TURN_WHEEL")) {
      float turnAngle = actions["TURN_WHEEL"];
      setWheelAngle(turnAngle);
    }
  }
}

UMAP<string, float> Host::getAutonomousActions(const vector<Vector2f>& path,
                                               const Simulation& simulation) {
  UMAP<string, float> output;
  if (node_id >= path.size()) node_id = 0;

  // set the timer to control time
  if (path.size() == 0) return output;
  int nextId;

  Vector2f vectogoal;
  nextId = node_id + 1;
  if (node_id >= path.size()) node_id = pre;
  if (nextId > path.size()) nextId = node_id;

  Vector2f nextpos = path[nextId];

  if (nextpos.get_distance(getPos()) <
      Globals::constant.BELIEF_TILE_SIZE * 0.3) {
    pre = node_id;
    node_id = nextId;
    nextId = node_id + 1;
  }

  if (nextId >= path.size()) nextId = node_id;

  // goalPos = path[nextId];
  // we finish the checking of end point
  vectogoal = path[nextId] - getPos();
  float wheel_angle = -vectogoal.get_angle_between(getDir());
  int sign = (wheel_angle < 0) ? -1 : 1;
  wheel_angle = std::min(abs(wheel_angle), max_wheel_angle);

  output["TURN_WHEEL"] = wheel_angle * sign;
  output["DRIVE_FORWARD"] = 1.0;
  // if (abs(wheel_angle) < 20) output["DRIVE_FORWARD"] = 1.0;
  // else if (abs(wheel_angle) < 45) output["DRIVE_FORWARD"] = 0.8;
  // else output["DRIVE_FORWARD"] = 0.5;

  return output;
}

void Host::makeObservation(const Simulation& simulation) {
  vector<Actor*> cars = simulation.getOtherCars();
  for (const auto& car : cars) {
    Vector2f obsv = dynamic_cast<Car*>(car)->getObservation();
    float obs = obsv.Length();
    obs = obs > 0 ? obs : 0;
    if (dynamic_cast<Car*>(car)->history.size() > 10) {
      dynamic_cast<Car*>(car)->history.pop();
    }
    dynamic_cast<Car*>(car)->history.push(obs);
  }
}

UMAP<string, float> Host::getAutonomousActions(const vector<Vector2f>& path,
                                               const Simulation& simulation,
                                               kdtree::kdtree<point<float>>* tree) {
  UMAP<string, float> output;
  if (node_id > path.size()) node_id = 0;

  static unsigned int timer = 0;
  static bool stop_flag = false;

  // set the timer to control time
  if (timer < 30 && stop_flag) {
    setVelocity(0.0);
    output["TURN_WHEEL"] = 0;
    output["DRIVE_FORWARD"] = 0;
    timer++;
    return output;
  }

  if (carInIntersection(simulation) && !stop_flag) {
    stop_flag = true;
    // setVelocity(0.0);
    output["TURN_WHEEL"] = 0;
    output["DRIVE_FORWARD"] = 0;
    timer = 0;
  }
  // finished checking the

  if (isCloseToOtherCar(simulation)) {
    output["TURN_WHEEL"] = 0;
    output["DRIVE_FORWARD"] = 0;
    return output;
  }

  if (path.size() == 0) return output;

  int nextId;
  //= node_id + 1;
  // if (node_id>=path.size()) node_id = pre;
  // if (nextId > path.size()) nextId = node_id;

  Vector2f vectogoal;

  // chek the kd tree
  if ((tree != nullptr) && (path.size() == tree->size())) {
    Vector2f mypos = getPos();
    vector<kdtree::node<point<float>>*> neighbors =
        tree->k_nearest(point<float>(mypos[0], mypos[1]), 2);
    point<float> p1 = neighbors[0]->point;
    point<float> p2 = neighbors[1]->point;
    //        Vector2f v1 = Vector2f(p1.x, p1.y);
    Vector2f v2 = Vector2f(p2.x, p2.y);
    //        Vector2f vectogoal1 = v1 - getPos();
    Vector2f vectogoal2 = v2 - getPos();
    float angle2 = abs(vectogoal2.get_angle_between(getDir()));
    if (angle2 < 90)
      nextId = p2.id;
    else
      nextId = p1.id;
  } else {
    nextId = node_id + 1;
    if (node_id >= path.size()) node_id = pre;
    if (nextId > path.size()) nextId = node_id;
  }

  Vector2f nextpos = path[nextId];

  if (nextpos.get_distance(getPos()) <
      Globals::constant.BELIEF_TILE_SIZE * 0.5) {
    pre = node_id;
    node_id = nextId;
    nextId = node_id + 1;
  }

  if (nextId >= path.size()) nextId = node_id;

  // we finish the checking of end point
  vectogoal = path[nextId] - getPos();
  float wheel_angle = -vectogoal.get_angle_between(getDir());
  int sign = (wheel_angle < 0) ? -1 : 1;
  wheel_angle = std::min(abs(wheel_angle), max_wheel_angle);

  output["TURN_WHEEL"] = wheel_angle * sign;
  output["DRIVE_FORWARD"] = 1.0;
  // if (abs(wheel_angle) < 20) output["DRIVE_FORWARD"] = 1.0;
  // else if (abs(wheel_angle) < 45) output["DRIVE_FORWARD"] = 0.8;
  // else output["DRIVE_FORWARD"] = 0.5;

  return output;
}

//************************************************************************
// class Car: methods and implementation
//************************************************************************

void Car::setup() {
  max_speed = 3.0;
  min_speed = 1.0;
  friction = 1;
  max_wheel_angle = 45;
  max_accler = 1.4;
  history = std::queue<float>();
  has_inference = false;
  inference = nullptr;
}

// tree is not used in this function
void Car::autonomousAction(const vector<Vector2f>& vec2, const Simulation& simulation, kdtree::kdtree<point<float>>* tree) {
  /*
   * here we have three choices to choose: normal, acc, dec
   */
  // set the timer to control time
  if (timer < 30 && stop_flag) {
    // setVelocity(0.0);
    timer++;
    return;
  }

  // Stop for a fixed duration at the stop sign or intersections.
  bool check = carInIntersection(simulation);
  if (check && !stop_flag) {
    stop_flag = true;
    accelerate(0);
    setWheelAngle(0);
    timer = 0;
    return;
  }

  if (isCloseToOtherCar(simulation)) {
    accelerate(0);
    setWheelAngle(0);
    return;
  }

  // unsigned int i = rand()%1;
  // assume it is not conservative for all drivers
  unsigned int i = 1;

  Actor* host = simulation.getHost();

  // conservative driver will yield
  if ((host->getPos().x < this->getPos().x + Actor::LENGTH * 4) &&
      (host->getPos().x > this->getPos().x)) {
    i = 0;
  }

  switch (i) {
    case 0:
      accelerate(friction);
      setWheelAngle(0);
      break;
    case 1:
      accelerate(max_accler);
      setWheelAngle(0);
      break;
    case 2:
      accelerate(max_accler * 0.25);
      setWheelAngle(0);
      break;
    default:
      break;
  }
}

void Car::autonomousAction(const vector<Vector2f>& vec2, const Simulation& simulation, int i) {
  // unsigned int i = rand()%1;
  // assume it is not conservative for all drivers
  switch (i) {
    case 0:
      accelerate(friction);
      setWheelAngle(0);
      break;
    case 1:
      accelerate(max_accler);
      setWheelAngle(0);
      break;
    default:
      break;
  }
}

Inference::MarginalInference* Car::getInference(int index,
                                                const Simulation& simulation) {
  if (!has_inference) {
    inference = new Inference::MarginalInference(index, simulation);
    has_inference = true;
    return inference;
  }

  return inference;
}
