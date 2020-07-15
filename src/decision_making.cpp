#include "decision_making.h"

const vector<std::string> DecisionAgent::hostActions = {
    "normal", "acc", "dec", "stop", "left", "right"};

const vector<std::string> DecisionAgent::otherActions = {"acc", "dec", "normal",
                                                         "stop"};

const unordered_map<std::string, float> DecisionAgent::actionReward = {
    {"normal", 3}, {"acc", 0},  {"dec", -1},
    {"stop", -1},  {"left", 0}, {"right", 0}};

// const unorder_mapd<std::string, std::pair<int,int> > command =
// {{"normal",{}}, "acc", "dec", "stop", "left", "right"};

/*
Evaluation function need to check many features

1. crash with the surroundings including othercars
2. distance to goal
3. distance to the neareast other cars, if it is two close, the score is less
*/
Model DecisionAgent::generateSuccessor(
    const Model& model, int agentIndex,
    const std::pair<std::string, vec2f>& actions) {
  Model newmodel = Model(model);
  // Car* car = newmodel.getCars()[agentIndex];
  std::string action = actions.first;
  ApplyAction(newmodel, agentIndex, action);
  return newmodel;
}

void DecisionAgent::ApplyAction(const Model& model, int agentIndex,
                                const std::string& action) {
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

unordered_map<std::string, vec2f> DecisionAgent::generateLegalActions(
    const Model& model, int agentIndex) {
  unordered_map<std::string, vec2f> legalActions;

  vector<std::string> actionlist;
  if (agentIndex == 0)
    actionlist = hostActions;
  else
    actionlist = otherActions;

  for (const std::string& action : actionlist) {
    Model newmodel = Model(model);
    Car* car = newmodel.getHost();
    ApplyAction(newmodel, agentIndex, action);

    if (action == "stop") {
      legalActions[action] = car->getPos();
      continue;
    }

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
      vec2f ndir = car->getDir();
      ndir = setToOne(ndir);
      vec2f pos(corToCenter(car->getPos() +
                            ndir * float(Globals::constant.BELIEF_TILE_SIZE)));
      legalActions[action] = pos;
    }
  }
  return legalActions;
}

float DecisionAgent::evaluationFunction(const Model& mo) {
  Model model(mo);

  float score;
  Car* mycar = model.getHost();
  if (model.checkCollision(mycar)) {
    score = -inf;
    return score;
  }

  Vector2f goal = mo.getFinish().getCenter();
  score = 0;
  score += 100 * (2 - abs(goal[0] - mycar->getPos()[0]) / 960);
  score += 100 * (2 - abs(mycar->getPos()[1] - goal[1]) / 100);

  float distance = inf;
  Car* obstaclecar = nullptr;
  vector<Car*> cars = model.getOtherCars();
  if (cars.size() == 0) return score;

  //# make sure it stay away from other cars
  for (Car* car : cars) {
    float cardis = manhaDistance(car->getPos(), mycar->getPos());
    if (cardis < distance) {
      distance = cardis;
      obstaclecar = car;
    }
  }

  if ((obstaclecar->getPos()[0] > mycar->getPos()[0]) &&
      (obstaclecar->getPos()[0] - mycar->getPos()[0]) <
          Globals::constant.BELIEF_TILE_SIZE * 2 &&
      abs(obstaclecar->getPos()[1] - mycar->getPos()[1]) < Car::WIDTH / 2)
    score -= 50;

  //    # # do not make turns when cars are very close to each other get too
  //    close to my car if ((abs(obstaclecar->getPos()[0] - mycar->getPos()[0]))
  //    < Car::WIDTH*4)
  //         score -=
  //         5*abs(mycar->getDir().get_angle_between(obstaclecar->getDir()))/45;
  vec2f pos = mycar->getPos();
  //     vec2f newdir = mycar->getDir();
  //     vec2f mdir = setToOne(newdir);

  int steps = 2;
  //
  //
  float delta = Globals::constant.BELIEF_TILE_SIZE / float(steps);

  for (int step = 0; step < steps; step++) {
    mycar->setVelocity(delta * step);
    mycar->update();
    if (model.checkCollision(mycar)) return -inf;

    float velocity = mycar->getVelocity().Length();

    float time = 0.0;
    if (velocity != 0) time = ((mycar->getPos() - pos).Length()) / velocity;

    for (Car* car : cars) {
      float newv = velocity * time;
      car->setVelocity(newv);
      if (model.checkCollision(mycar)) return -inf;
    }
  }

  return score;
}

float DecisionAgent::value(const Model& model, int agentindex, int dep) {
  if (dep == depth || model.checkCollision(model.getHost()))
    return evaluationFunction(model);

  agentindex = agentindex % model.getCars().size();
  if (agentindex == 0) {
    dep += 1;
    return maxvalue(model, agentindex, dep);
  } else
    return expecvalue(model, agentindex, dep);
  return 0.0;
}

float DecisionAgent::maxvalue(const Model& gameState, int agentindex, int dep) {
  if (dep == depth) return evaluationFunction(gameState);

  float score = -inf;
  std::unordered_map<std::string, vec2f> actions =
      generateLegalActions(gameState, agentindex);
  for (const auto& iter : actions) {
    std::pair<std::string, vec2f> item(iter.first, iter.second);
    Model model = generateSuccessor(gameState, agentindex, item);
    score = max(score, value(model, agentindex + 1, depth) +
                           actionReward.at(iter.first));
  }
  return score;
}

float DecisionAgent::expecvalue(const Model& model, int agentindex, int depth) {
  std::pair<std::string, vec2f> action;

  float score = 0.0;
  std::unordered_map<std::string, vec2f> actions =
      generateLegalActions(model, agentindex);

  std::vector<std::string> acts;
  for (const auto& iter : actions) acts.push_back(iter.first);

  // float prob = 1/float(actions.size());
  std::string act = "normal";
  if (actions.count(act) != 0)
    action = make_pair(act, actions[act]);
  else {
    int i = rand() % acts.size();
    std::string act = acts[i];
    action = make_pair(act, actions[act]);
  }
  Model newState = generateSuccessor(model, agentindex, action);
  score =
      value(newState, agentindex + 1, depth) + actionReward.at(action.first);

  return score;
}

std::pair<std::string, vec2f> DecisionAgent::getAction(const Model& model) {
  std::string bestAction = "stop";
  // int numAgents = model.getCars().size();
  std::unordered_map<std::string, vec2f> actions =
      generateLegalActions(model, 0);

  vector<std::string> keys;
  for (const auto& iter : actions) keys.push_back(iter.first);

  float score = -inf;
  if (actions.size() == 1)
    return std::pair<std::string, vec2f>(keys[0], actions.at(keys[0]));

  for (const auto& action : actions) {
    std::string key = action.first;
    std::pair<std::string, vec2f> item(key, actions.at(key));
    Model newgamestates = generateSuccessor(model, 0, item);

    float newscore = value(newgamestates, 1, 0) + actionReward.at(key);

    if (score < newscore) {
      score = newscore;
      bestAction = key;
    }
  }
  std::pair<std::string, vec2f> result(bestAction, actions.at(bestAction));
  return result;
};

bool DecisionAgent::isCloseToOtherCar(Car* mycar, const Model& model) const {
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
  if (abs(obstaclecar->getPos()[0] - mycar->getPos()[0]) <
          Globals::constant.BELIEF_TILE_SIZE * 1.5 &&
      abs(obstaclecar->getPos()[1] - mycar->getPos()[1]) < Car::WIDTH)
    return true;
  return false;
}
