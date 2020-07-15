#include "decision_making.h"

vector<std::string> DecisionMaker::hostActions = {
  "normal",
  "left",
  "right"
};

vector<std::string> DecisionMaker::otherActions = {
  "acc",
  "dec",
  "normal",
  "stop"
};

unordered_map<std::string, float> DecisionMaker::actionReward = {
  {"normal", 3},
  {"acc", 0},
  {"dec", -1},
  {"stop", -1},
  {"left", 0},
  {"right", 0}
};

/*Evaluation function need to check many features
1. crash with the surroundings including othercars
2. distance to goal
3. distance to the neareast other cars, if it is two close, the score is less
*/
vector<vector<vec2f>>& DecisionMaker::generatePaths(const Simulation& mod, vector<string>& legalactions) {
  if (paths.size() > 0) paths.clear();

  Simulation simulation = mod;
  Car* host = simulation.getHost();
  // vec2f ndir = host->getDir();

  for (int i = 0; i < legalactions.size(); i++) {
    vec2f ndir = vec2f(1, 1);

    if (legalactions[i] == "normal") continue;
    if (legalactions[i] == "right") {
      ndir = vec2f(1, -1);
    }

    vec2f pos(host->getPos() + ndir * float(Globals::constant.BELIEF_TILE_SIZE));

    vec2f Pos(pos.x + 50, host->getPos().y);
    SEARCH::Search search(&simulation, Pos);
    vector<vec2f> path = search.path();
    paths.push_back(path);

    for (float deltax = 0; deltax < 80; deltax += 10) {
      vec2f Pos(pos.x + deltax, pos.y);
      SEARCH::Search search(&simulation, Pos);
      vector<vec2f> path = search.path();
      paths.push_back(path);
    }
  }
  return paths;
}

void DecisionMaker::ApplyAction(const Simulation& simulation, int agentIndex, const std::string& action) {
  Car* car = simulation.getCars()[agentIndex];
  if (action == "normal") {
    car->accelerate(car->friction);
    car->setWheelAngle(0);
  }
  if (action == "acc") {
    car->accelerate(car->maxaccler);
    car->setWheelAngle(0);
  }
  if (action == "dec") {
    car->accelerate(car->maxaccler * 0.25);
    car->setWheelAngle(0);
  }
  if (action == "stop") {
    car->setVelocity(0);
    car->setWheelAngle(0);
  }
  if (action == "left") {
    car->setWheelAngle(-45);
    ;
    car->accelerate(car->maxaccler);
  }
  if (action == "right") {
    car->setWheelAngle(45);
    car->accelerate(car->maxaccler);
  }
  car->update();
}

float DecisionMaker::evaluationPath(const Simulation& sim, const vector<vec2f>& path, vector<int>& car_intentions) {
  Simulation simulation(sim);
  float score = 0.0;
  Car* ego_car = simulation.getHost();
  vec2f carpos = ego_car->getPos();

  while (abs(carpos.x - path[path.size() - 1].x) > 5) {
    ego_car->autonomousAction2(path, simulation);
    ego_car->update();

    for (int i = 0; i < simulation.getOtherCars().size(); i++) {
      Car* car = simulation.getOtherCars()[i];
      car->autonomousAction2(path, simulation, car_intentions[i]);
      car->update();
    }

    if (simulation.checkCollision(ego_car)) return -inf;

    carpos = ego_car->getPos();
  }

  ego_car->setPos(path[path.size() - 1]);

  if (simulation.checkCollision(ego_car)) return -inf;

  if (isCloseToOtherCar(ego_car, simulation)) return -inf;

  Vector2f goal = sim.getFinish().getCenter();
  score += 100 * (1 - abs(goal[0] - ego_car->getPos()[0]) / 960);
  score += 100 * (1 - abs(ego_car->getPos()[1] - goal[1]) / 100);

  return score;
}

bool DecisionMaker::getPath(const Simulation& simulation, vector<vec2f>& final_path, vector<int>& car_intentions) {
  // std::string bestAction = "stop";
  // int numAgents = simulation.getCars().size();
  vector<string> legalactions = generateLegalActions(simulation);
  generatePaths(simulation, legalactions);
  int index = 0;
  float bestscore = -inf;
  float score = 0.0;

  for (int i = 0; i < paths.size(); i++) {
    score = evaluationPath(simulation, paths[i], car_intentions);
    if (bestscore < score) {
      index = i;
      bestscore = score;
    }
  }

  final_path = paths[index];

  // index = 0 means, it can't find its next path to go
  if (index == 0) return false;
  return true;
};

vector<string> DecisionMaker::generateLegalActions(const Simulation& simulation) {
  vector<string> actionlist = hostActions;
  vector<string> legalactions;

  for (const std::string& action : actionlist) {
    Simulation newmodel = Simulation(simulation);
    Car* car = newmodel.getHost();

    if (action == "left") {
      car->setWheelAngle(45);
    }
    else if (action == "right") {
      car->setWheelAngle(-45);
    }

    car->setVelocity(sqrt(2) / 2 * float(Globals::constant.BELIEF_TILE_SIZE));
    car->update();

    vector<vec2f> bounds = car->getBounds();

    bool isinBound = true;
    for (const vec2f& point : bounds) {
      if (!newmodel.inBounds(point[0], point[1])) {
        isinBound = false;
        break;
      }
    }

    // check if it is inbound
    if (isinBound) {
      legalactions.push_back(action);
    }
  }
  return legalactions;
}

bool DecisionMaker::isCloseToOtherCar(Car* ego_car, const Simulation& simulation) const {
  vector<Car*> cars = simulation.getOtherCars();
  if (cars.size() == 0) return false;

  Car* obstaclecar = nullptr;
  float distance = inf;

  for (Car* car : cars) {
    float cardis = manhattanDistance(car->getPos(), ego_car->getPos());
    if (cardis < distance) {
      distance = cardis;
      obstaclecar = car;
    }
  }

  if (abs(obstaclecar->getPos()[0] - ego_car->getPos()[0]) < Globals::constant.BELIEF_TILE_SIZE * 1.2 &&
      abs(obstaclecar->getPos()[1] - ego_car->getPos()[1]) < Car::WIDTH) {
    return true;
  }
  return false;
}

bool DecisionMaker::isChangeRequired(Car* ego_car, const Simulation& simulation) {
  Car* host = simulation.getHost();
  vec2f goal = simulation.getFinish().getCenter();

  if (abs(host->getPos().y - goal.y) < 5) return false;
  return true;
}
