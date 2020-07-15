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
vector<vector<vec2f>>& DecisionMaker::generatePaths(const Model& mod, vector<string>& legalactions) {
  if (paths.size() > 0) paths.clear();

  Model model = mod;
  Car* host = model.getHost();
  // vec2f ndir = host->getDir();

  for (int i = 0; i < legalactions.size(); i++) {
    vec2f ndir = vec2f(1, 1);

    if (legalactions[i] == "normal") continue;
    if (legalactions[i] == "right") {
      ndir = vec2f(1, -1);
    }

    vec2f pos(host->getPos() + ndir * float(Globals::constant.BELIEF_TILE_SIZE));

    vec2f Pos(pos.x + 50, host->getPos().y);
    SEARCH2::Search search(&model, Pos);
    vector<vec2f> path = search.path();
    paths.push_back(path);

    for (float deltax = 0; deltax < 80; deltax += 10) {
      vec2f Pos(pos.x + deltax, pos.y);
      SEARCH2::Search search(&model, Pos);
      vector<vec2f> path = search.path();
      paths.push_back(path);
    }
  }
  return paths;
}

vector<vector<vec2f>>& DecisionMaker::generatePaths2(const Model& mod, vector<string>& legalactions) {
  if (paths.size() > 0) paths.clear();

  Model model = mod;
  Car* host = model.getHost();
  // vec2f ndir = host->getDir();

  for (int i = 0; i < legalactions.size(); i++) {
    vec2f ndir = vec2f(1, 1);

    if (legalactions[i] == "normal") continue;
    if (legalactions[i] == "right") {
      ndir = vec2f(1, -1);
    }

    vec2f pos(host->getPos() + ndir * float(Globals::constant.BELIEF_TILE_SIZE));

    vec2f Pos(pos.x + 50, host->getPos().y);
    SEARCH::Search search(&model, Pos);
    vector<vec2f> path = search.path();
    paths.push_back(path);

    for (float deltax = 0; deltax < 80; deltax += 10) {
      vec2f Pos(pos.x + deltax, pos.y);
      SEARCH::Search search(&model, Pos);
      vector<vec2f> path = search.path();
      paths.push_back(path);
    }
  }
  return paths;
}

void DecisionMaker::ApplyAction(const Model& model, int agentIndex, const std::string& action) {
  Car* car = model.getCars()[agentIndex];
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

float DecisionMaker::evaluationPath(const Model& mo, const vector<vec2f>& path, vector<int>& carintentions) {
  Model model(mo);
  float score = 0.0;
  Car* mycar = model.getHost();
  vec2f carpos = mycar->getPos();

  while (abs(carpos.x - path[path.size() - 1].x) > 5) {
    mycar->autonomousAction2(path, model);
    mycar->update();

    for (int i = 0; i < model.getOtherCars().size(); i++) {
      Car* car = model.getOtherCars()[i];
      car->autonomousAction2(path, model, carintentions[i]);
      car->update();
    }

    if (model.checkCollision(mycar)) return -inf;

    carpos = mycar->getPos();
  }

  mycar->setPos(path[path.size() - 1]);

  if (model.checkCollision(mycar)) return -inf;

  if (isCloseToOtherCar(mycar, model)) return -inf;

  Vector2f goal = mo.getFinish().getCenter();
  score += 100 * (1 - abs(goal[0] - mycar->getPos()[0]) / 960);
  score += 100 * (1 - abs(mycar->getPos()[1] - goal[1]) / 100);

  return score;
}

bool DecisionMaker::getPath(const Model& model, vector<vec2f>& mypath, vector<int>& carintentions) {
  // std::string bestAction = "stop";
  // int numAgents = model.getCars().size();
  vector<string> legalactions = generateLegalActions(model);
  generatePaths(model, legalactions);
  int index = 0;
  float bestscore = -inf;
  float score = 0.0;

  for (int i = 0; i < paths.size(); i++) {
    score = evaluationPath(model, paths[i], carintentions);
    if (bestscore < score) {
      index = i;
      bestscore = score;
    }
  }

  mypath = paths[index];

  // index = 0 means, it can't find its next path to go
  if (index == 0) return false;
  return true;
};

bool DecisionMaker::getPath2(const Model& model, vector<vec2f>& mypath, vector<int>& carintentions) {
  // std::string bestAction = "stop";
  // int numAgents = model.getCars().size();
  vector<string> legalactions = generateLegalActions(model);
  generatePaths2(model, legalactions);
  int index = 0;
  float bestscore = -inf;
  float score = 0.0;

  for (int i = 0; i < paths.size(); i++) {
    score = evaluationPath(model, paths[i], carintentions);
    if (bestscore < score) {
      index = i;
      bestscore = score;
    }
  }

  mypath = paths[index];

  // index = 0 means, it can't find its next path to go
  if (index == 0) return false;
  return true;
};

vector<string> DecisionMaker::generateLegalActions(const Model& model) {
  vector<string> actionlist = hostActions;
  vector<string> legalactions;

  for (const std::string& action : actionlist) {
    Model newmodel = Model(model);
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

bool DecisionMaker::isCloseToOtherCar(Car* mycar, const Model& model) const {
  vector<Car*> cars = model.getOtherCars();
  if (cars.size() == 0) return false;

  Car* obstaclecar = nullptr;
  float distance = inf;

  for (Car* car : cars) {
    float cardis = manhattanDistance(car->getPos(), mycar->getPos());
    if (cardis < distance) {
      distance = cardis;
      obstaclecar = car;
    }
  }

  if (abs(obstaclecar->getPos()[0] - mycar->getPos()[0]) < Globals::constant.BELIEF_TILE_SIZE * 1.2 &&
      abs(obstaclecar->getPos()[1] - mycar->getPos()[1]) < Car::WIDTH) {
    return true;
  }
  return false;
}

bool DecisionMaker::isChangeRequired(Car* mycar, const Model& model) {
  Car* host = model.getHost();
  vec2f goal = model.getFinish().getCenter();

  if (abs(host->getPos().y - goal.y) < 5) return false;
  return true;
}
