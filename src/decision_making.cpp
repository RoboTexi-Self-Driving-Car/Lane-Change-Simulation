#include "decision_making.h"

vector<std::string> DecisionMaker::m_host_actions = {
  "normal",
  "left",
  "right"
};

/*
  Evaluation function need to check many features
  1. crash with the surroundings including other cars
  2. distance to goal
  3. distance to the neareast other cars, if it is two close, the score is less
*/
vector<vector<Vec2f>>& DecisionMaker::generatePaths(const Simulation& simulation, vector<string>& legal_actions) {
  if (paths.size() > 0) paths.clear();

  Simulation sim = simulation;
  Actor* host = sim.getHost();
  // Vec2f ndir = host->getDir();

  for (int i = 0; i < legal_actions.size(); i++) {
    Vec2f ndir = Vec2f(1, 1);

    if (legal_actions[i] == "normal") continue;
    if (legal_actions[i] == "right") {
      ndir = Vec2f(1, -1);
    }

    Vec2f host_pos(host->getPos() + ndir * float(Globals::constant.BELIEF_TILE_SIZE));

    Vec2f des_pos(host_pos.x + 50, host->getPos().y);
    SEARCH::Search search(&sim, des_pos);
    vector<Vec2f> path = search.path();
    paths.push_back(path);

    for (float deltax = 0; deltax < 80; deltax += 10) {
      Vec2f des_pos(host_pos.x + deltax, host_pos.y);
      SEARCH::Search search(&sim, des_pos);
      vector<Vec2f> path = search.path();
      paths.push_back(path);
    }
  }
  return paths;
}

void DecisionMaker::applyAction(const Simulation& simulation, int index, const std::string& action) {
  Actor* car = simulation.getAllCars()[index];
  if (action == "normal") {
    car->accelerate(car->friction);
    car->setWheelAngle(0);
  }
  if (action == "acc") {
    car->accelerate(car->max_accler);
    car->setWheelAngle(0);
  }
  if (action == "dec") {
    car->accelerate(car->max_accler * 0.25);
    car->setWheelAngle(0);
  }
  if (action == "stop") {
    car->setVelocity(0);
    car->setWheelAngle(0);
  }
  if (action == "left") {
    car->setWheelAngle(-45);
    car->accelerate(car->max_accler);
  }
  if (action == "right") {
    car->setWheelAngle(45);
    car->accelerate(car->max_accler);
  }
}

float DecisionMaker::evaluatePath(const Simulation& simulation, const vector<Vec2f>& path, vector<int>& car_intentions) {
  Simulation sim(simulation);
  float score = 0.0;
  Actor* host = sim.getHost();
  Vec2f host_pos = host->getPos();

  // Criteria 1: collision checking
  while (abs(host_pos.x - path[path.size() - 1].x) > 5) {
    // check in the each update (position) if there is collision
    host->autonomousAction(path, sim, 1);
    host->update();

    for (int i = 0; i < sim.getOtherCars().size(); i++) {
      Actor* car = sim.getOtherCars()[i];
      car->autonomousAction(path, sim, car_intentions[i]);
      car->update();
    }

    if (sim.checkCollision(host)) return -inf;

    host_pos = host->getPos();
  }

  // check in the last update (position) if there is collision
  host->setPos(path[path.size() - 1]);
  if (sim.checkCollision(host)) return -inf;

  // even if there is no collision but still need to avoid too close
  if (isCloseToOtherCar(host, sim)) return -inf;

  Vector2f goal = simulation.getGoal().getCenter();

  // Criteria 2: distance to goal
  // The final position gets closer to the goal position,
  // the path gets higher score.
  score += 100 * (1 - abs(goal[0] - host->getPos()[0]) / 960); // score of x in [0, 100]
  score += 100 * (1 - abs(goal[1] - host->getPos()[1]) / 100); // score of y in [0, 100]

  return score;
}

bool DecisionMaker::getPath(const Simulation& simulation, vector<Vec2f>& final_path, vector<int>& car_intentions) {
  // std::string bestAction = "stop";
  // int num_cars = simulation.getAllCars().size();
  vector<string> legal_actions = generateLegalActions(simulation);
  generatePaths(simulation, legal_actions);
  int best_index = 0;
  float best_score = -inf;
  float score = 0.0;

  for (int i = 0; i < paths.size(); i++) {
    score = evaluatePath(simulation, paths[i], car_intentions);
    if (score > best_score) {
      best_index = i;
      best_score = score;
    }
  }

  final_path = paths[best_index];

  // best_index = 0 means, it can't find its next path to go
  if (best_index == 0) return false;
  return true;
};

vector<string> DecisionMaker::generateLegalActions(const Simulation& simulation) {
  vector<string> action_list = m_host_actions;
  vector<string> legal_actions;

  for (const std::string& action : action_list) {
    Simulation sim(simulation);
    Actor* host = sim.getHost();

    if (action == "left") {
      host->setWheelAngle(45);
    }
    else if (action == "right") {
      host->setWheelAngle(-45);
    }

    host->setVelocity(sqrt(2) / 2 * float(Globals::constant.BELIEF_TILE_SIZE));
    host->update();

    vector<Vec2f> bounds = host->getBounds();

    // check if it is still inbound of the lanes
    bool inBound = true;
    for (const Vec2f& point : bounds) {
      if (!sim.inBounds(point[0], point[1])) {
        inBound = false;
        break;
      }
    }

    if (inBound) {
      legal_actions.push_back(action);
    }
  }
  return legal_actions;
}

bool DecisionMaker::isCloseToOtherCar(Actor* host, const Simulation& simulation) const {
  vector<Actor*> cars = simulation.getOtherCars();
  if (cars.size() == 0) return false;

  Actor* obstacle_car = nullptr;
  float distance = inf;

  for (Actor* car : cars) {
    float dist = manhattanDistance(car->getPos(), host->getPos());
    if (dist < distance) {
      distance = dist;
      obstacle_car = car;
    }
  }

  if (abs(obstacle_car->getPos()[0] - host->getPos()[0]) < Globals::constant.BELIEF_TILE_SIZE * 1.2 &&
      abs(obstacle_car->getPos()[1] - host->getPos()[1]) < Actor::WIDTH) {
    return true;
  }
  return false;
}

bool DecisionMaker::isChangeRequired(const Simulation& simulation) {
  Actor* host = simulation.getHost();
  Vec2f goal = simulation.getGoal().getCenter();

  if (abs(host->getPos().y - goal.y) < 5) return false;
  return true;
}
