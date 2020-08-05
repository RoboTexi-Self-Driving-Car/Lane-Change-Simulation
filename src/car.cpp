#include "car.h"

//************************************************************************
// class Car: methods and implementation
//************************************************************************

const float Car::RADIUS = sqrt(pow(Car::LENGTH, 2) + pow(Car::WIDTH, 2));

void Car::init(const string& dir) {
  pii p = direction[dir];
  this->dir = Vector2f(p.first, p.second);
  wheelAngle = 0;
  setup();
}

void Car::init() {
  wheelAngle = 0;
  setup();
}

void Car::setup() {
  maxSpeed = 5.0;
  friction = 0.5;
  maxWheelAngle = 130.0;
  maxaccler = 2.0;
  minSpeed = 1;
}

void Car::turnCarTowardsWheels() {
  if (velocity.Length() > 0.0) {
    velocity.rotate(wheelAngle);
    dir = Vector2f(velocity[0], velocity[1]);
    dir.normalized();
  }
}

void Car::update() {
  turnCarTowardsWheels();
  pos += velocity;
  turnWheelsTowardsStraight();
  applyFriction();
}

void Car::decellerate(float amount) {
  float speed = velocity.Length();

  if (speed < minSpeed) {
    speed = minSpeed;
    return;
  }

  Vector2f frictionVec = velocity.get_reflection();
  frictionVec.normalized();
  frictionVec *= amount;
  velocity += frictionVec;
  float angle = velocity.get_angle_between(frictionVec);

  if (abs(angle) < 180) velocity = Vector2f(0, 0);
}

void Car::setWheelAngle(float angle) {
  wheelAngle = angle;

  if (wheelAngle <= -maxWheelAngle) wheelAngle = -maxWheelAngle;
  if (wheelAngle >= maxWheelAngle) wheelAngle = maxWheelAngle;
}

void Car::accelerate(float amount) {
  amount = std::min(amount, maxaccler);

  if (amount < 0) decellerate(amount);
  if (amount == 0) return;

  Vector2f acceleration = Vector2f(dir[0], dir[1]);
  acceleration.normalized();
  acceleration *= amount;
  velocity += acceleration;

  if (velocity.Length() >= maxSpeed) {
    velocity.normalized();
    velocity *= maxSpeed;
  }
}

vector<Vector2f> Car::getBounds() {
  dir.normalized();
  Vector2f perpDir = dir.perpendicular();

  vector<Vector2f> bounds;
  bounds.push_back(pos + dir * float(LENGTH / 2) + perpDir * float(WIDTH / 2));
  bounds.push_back(pos + dir * float(LENGTH / 2) - perpDir * float(WIDTH / 2));
  bounds.push_back(pos - dir * float(LENGTH / 2) + perpDir * float(WIDTH / 2));
  bounds.push_back(pos - dir * float(LENGTH / 2) - perpDir * float(WIDTH / 2));

  return bounds;
}

vector<Vector2f> Car::getBounds(Car& car, float LEN, float WID) {
  Vector2f normalDir = normalized(car.getDir());
  Vector2f perpDir = normalDir.perpendicular();

  vector<Vector2f> bounds;
  bounds.push_back(pos + dir * float(LEN / 2) + perpDir * float(WID / 2));
  bounds.push_back(pos + dir * float(LEN / 2) - perpDir * float(WID / 2));
  bounds.push_back(pos - dir * float(LEN / 2) + perpDir * float(WID / 2));
  bounds.push_back(pos - dir * float(LEN / 2) - perpDir * float(WID / 2));

  return bounds;
}

//#
//http://www.gamedev.net/page/resources/_/technical/game-programming/2d-rotated-rectangle-collision-r2604
bool Car::collides(const Vector2f& otherPos,
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
void Car::setVelocity(float amount) {
  Vector2f ve = Vector2f(dir[0], dir[1]);
  ve.normalized();
  ve *= amount;
  velocity = ve;
}

// check car is in instersection
bool Car::carInintersection(const Simulation& simulation) {
  vector<Vector2f> bounds = getBounds();
  for (const auto& point : bounds) {
    if (simulation.inIntersection(point[0], point[1])) return true;
  }
  return false;
}

bool Car::isCloseToOtherCar(const Simulation& simulation) const {
  //### check the master car is close to others
  vector<Car*> cars = simulation.getCars();
  if (cars.size() == 0) return false;
  const Car* obstaclecar = nullptr;
  float distance = 9999999;
  for (const Car* car : cars) {
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
      (abs(obstaclecar->getPos()[1] - getPos()[1]) < Car::WIDTH / 2))
    return true;
  return false;
}

//************************************************************************
// class Host: methods and implementation
//************************************************************************

void Host::setup() {
  maxSpeed = 3.0;
  friction = 1;
  maxWheelAngle = 45;
  maxaccler = 1.5;
  minSpeed = 1;
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
    accelerate(maxWheelAngle * percent);
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

  UMAP<string, float> actions = getAutonomousActions2(path, simulation);

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
    accelerate(maxWheelAngle * percent);
    if (actions.count("TURN_WHEEL")) {
      float turnAngle = actions["TURN_WHEEL"];
      setWheelAngle(turnAngle);
    }
  }
}

UMAP<string, float> Host::getAutonomousActions2(const vector<Vector2f>& path,
                                                const Simulation& simulation) {
  UMAP<string, float> output;
  if (nodeId >= path.size()) nodeId = 0;

  // set the timer to control time
  if (path.size() == 0) return output;
  int nextId;

  Vector2f vectogoal;
  nextId = nodeId + 1;
  if (nodeId >= path.size()) nodeId = pre;
  if (nextId > path.size()) nextId = nodeId;

  Vector2f nextpos = path[nextId];

  if (nextpos.get_distance(getPos()) <
      Globals::constant.BELIEF_TILE_SIZE * 0.3) {
    pre = nodeId;
    nodeId = nextId;
    nextId = nodeId + 1;
  }

  if (nextId >= path.size()) nextId = nodeId;

  //        goalPos = path[nextId];
  // we finish the checking of end point
  vectogoal = path[nextId] - getPos();
  float wheelAngle = -vectogoal.get_angle_between(getDir());
  int sign = (wheelAngle < 0) ? -1 : 1;
  wheelAngle = std::min(abs(wheelAngle), maxWheelAngle);

  output["TURN_WHEEL"] = wheelAngle * sign;
  output["DRIVE_FORWARD"] = 1.0;
  //    if (abs(wheelAngle) < 20) output["DRIVE_FORWARD"] = 1.0;
  //    else if (abs(wheelAngle) < 45) output["DRIVE_FORWARD"] = 0.8;
  //    else output["DRIVE_FORWARD"] = 0.5;

  return output;
}

void Host::makeObse(const Simulation& state) {
  vector<Car*> cars = state.getOtherCars();
  for (const auto& car : cars) {
    Vector2f obsv = dynamic_cast<Agent*>(car)->getObserv();
    float obs = obsv.Length();
    obs = obs > 0 ? obs : 0;
    if (dynamic_cast<Agent*>(car)->history.size() == 11)
      dynamic_cast<Agent*>(car)->history.pop();
    dynamic_cast<Agent*>(car)->history.push(obs);
  }
}

UMAP<string, float> Host::getAutonomousActions(
    const vector<Vector2f>& path, const Simulation& simulation,
    kdtree::kdtree<point<float>>* tree) {
  UMAP<string, float> output;
  if (nodeId > path.size()) nodeId = 0;

  static unsigned int timer = 0;
  static bool stopflag = false;

  // set the timer to control time
  if (timer < 30 && stopflag) {
    setVelocity(0.0);
    output["TURN_WHEEL"] = 0;
    output["DRIVE_FORWARD"] = 0;
    timer++;
    return output;
  }

  if (carInintersection(simulation) && !stopflag) {
    stopflag = true;
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
  //= nodeId + 1;
  // if (nodeId>=path.size()) nodeId = pre;
  // if (nextId > path.size()) nextId = nodeId;

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
    nextId = nodeId + 1;
    if (nodeId >= path.size()) nodeId = pre;
    if (nextId > path.size()) nextId = nodeId;
  }

  Vector2f nextpos = path[nextId];

  if (nextpos.get_distance(getPos()) <
      Globals::constant.BELIEF_TILE_SIZE * 0.5) {
    pre = nodeId;
    nodeId = nextId;
    nextId = nodeId + 1;
  }

  if (nextId >= path.size()) nextId = nodeId;

  // we finish the checking of end point
  vectogoal = path[nextId] - getPos();
  float wheelAngle = -vectogoal.get_angle_between(getDir());
  int sign = (wheelAngle < 0) ? -1 : 1;
  wheelAngle = std::min(abs(wheelAngle), maxWheelAngle);

  output["TURN_WHEEL"] = wheelAngle * sign;
  output["DRIVE_FORWARD"] = 1.0;
  // if (abs(wheelAngle) < 20) output["DRIVE_FORWARD"] = 1.0;
  // else if (abs(wheelAngle) < 45) output["DRIVE_FORWARD"] = 0.8;
  // else output["DRIVE_FORWARD"] = 0.5;

  return output;
}

//************************************************************************
// class Agent: methods and implementation
//************************************************************************

void Agent::setup() {
  maxSpeed = 3.0;
  friction = 1;
  maxWheelAngle = 45;
  maxaccler = 1.4;
  minSpeed = 1;
  history = std::queue<float>();
  hasinference = false;
  inference = nullptr;
}

void Agent::autonomousAction(const vector<Vector2f>& vec2, const Simulation& simulation, kdtree::kdtree<point<float>>* tree) {
  /*
   * here we have three choices to choose: normal, acc, dec
   */
  // set the timer to control time
  if (timer < 30 && stopflag) {
    // setVelocity(0.0);
    timer++;
    return;
  }
  //
  bool check = carInintersection(simulation);
  if (check && !stopflag) {
    stopflag = true;
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

  Car* host = simulation.getHost();

  // conservative driver will yield
  if ((host->getPos().x < this->getPos().x + Car::LENGTH * 4) &&
      (host->getPos().x > this->getPos().x))
    i = 0;
  switch (i) {
    case 0:
      accelerate(friction);
      setWheelAngle(0);
      break;
    case 1:
      accelerate(maxaccler);
      setWheelAngle(0);
      break;
    case 2:
      accelerate(maxaccler * 0.25);
      setWheelAngle(0);
      break;
    default:
      break;
  }
}

void Agent::autonomousAction(const vector<Vector2f>& vec2, const Simulation& simulation, int i) {
  // unsigned int i = rand()%1;
  // assume it is not conservative for all drivers
  switch (i) {
    case 0:
      accelerate(friction);
      setWheelAngle(0);
      break;
    case 1:
      accelerate(maxaccler);
      setWheelAngle(0);
      break;
    default:
      break;
  }
}

Inference::MarginalInference* Agent::getInference(int index,
                                                  const Simulation& state) {
  if (!hasinference) {
    inference = new Inference::MarginalInference(index, state);
    hasinference = true;
    return inference;
  }

  return inference;
}
