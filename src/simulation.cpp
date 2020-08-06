#include "simulation.h"

//************************************************************************
// class Simulation: method implementations
//************************************************************************

Simulation::Simulation(Layout& lay) : layout(lay) {
  initLines();
  initBlocks();
  initGraphs();
  initIntersections();
  // initOtherCars();

  int startX = layout.getStartX();
  int startY = layout.getStartY();
  string startDir = layout.getHostDir();
  vector<int> goal_data = layout.getGoal();
  goal = new Block(goal_data);
  host = new Host(Vector2f(startX, startY), startDir, Vector2f(0.0, 0.0));
  all_cars.push_back(host);

  for (vector<int> other : layout.getOtherData()) {
    Actor* othercar =
        new Car(Vector2f(other[0], other[1]), "east", Vector2f(0.0, 0.0));
    other_cars.push_back(othercar);
    all_cars.push_back(othercar);
  }

  car2index = UMAP<size_t, int>();

  for (int i = 0; i < other_cars.size(); i++) {
    car2index.insert({(size_t)other_cars[i], i});
  }
}

Simulation::Simulation(const Simulation& simulation) : layout(simulation.layout) {
  goal = simulation.goal;
  blocks = simulation.blocks;
  lines = simulation.lines;
  interSections = simulation.interSections;
  agentGraph = simulation.agentGraph;
  hostGraph = simulation.hostGraph;
  allGraph = simulation.allGraph;
  host = new Host(*simulation.getHost());
  host->setup();
  all_cars.push_back(host);

  for (Actor* car : simulation.getOtherCars()) {
    Actor* othercar = new Car(*car);
    othercar->setup();
    other_cars.push_back(othercar);
    all_cars.push_back(othercar);
  }
}

Simulation::~Simulation() {
  if (all_cars.size() != 0) {
    while (all_cars.size() != 0) {
      delete all_cars.back();
      all_cars.pop_back();
    }
  }
}

void Simulation::setHost(Actor* car) {
  vector<Actor*> cars;
  cars.push_back(car);
  for (auto c : cars) {
    if (typeid(*c) != typeid(*host)) cars.push_back(c);
  }
  all_cars = cars;
}

void Simulation::clearBlocks(vector<Block*>& bloc) {
  if (bloc.size() != 0) {
    while (bloc.size() != 0) {
      delete bloc.back();
      bloc.pop_back();
    }
  }
}

void Simulation::initBlocks() {
  for (vector<int> blockData : layout.getBlockData()) {
    blocks.push_back(new Block(blockData));
  }
}

void Simulation::initLines() {
  for (vector<int> lineData : layout.getLineData())
    lines.push_back(new Line(lineData));
}

void Simulation::initGraphs() {
  for (vector<int> data : layout.getHostGraph()) {
    Block* hostgraph = new Block(data);
    hostGraph.push_back(hostgraph);
    allGraph.push_back(hostgraph);
  }

  for (vector<int> data : layout.getAgentGraph()) {
    Block* agentgraph = new Block(data);
    agentGraph.push_back(agentgraph);
    allGraph.push_back(agentgraph);
  }
}

void Simulation::initIntersections() {
  for (vector<int> blockData : layout.getIntersectionData()) {
    Block* inter = new Block(blockData);
    interSections.push_back(inter);
    allGraph.push_back(inter);
  }
}

bool Simulation::checkVictory() const {

  vector<Vector2f> bounds = host->getBounds();
  for (Vector2f point : bounds) {
    if (goal->containsPoint(point[0], point[1])) return true;
  }

  return false;
}

bool Simulation::checkCollision(Actor* car) const {
  vector<Vector2f> bounds = car->getBounds();
  for (Vector2f point : bounds) {
    if (!inBounds(point.x, point.y)) return true;
  }

  for (Actor* othercar : all_cars) {
    if (othercar == car) continue;
    if (othercar->collides(car->getPos(), bounds)) return true;
  }

  return false;
}

bool Simulation::inBounds(float x, float y) const {
  if (x < 0 || x >= getWidth()) return false;
  if (y < 0 || y >= getHeight()) return false;
  for (const auto& it : blocks) {
    if (it->containsPoint(x, y)) return false;
  }
  return true;
}

bool Simulation::inBoundsLarger(float x, float y) const {
  if (x < 0 || x >= getWidth()) return false;
  if (y < 0 || y >= getHeight()) return false;
  for (const auto& it : blocks)
    if (it->containsPointLarger(x, y)) return false;
  return true;
}

bool Simulation::inIntersection(float x, float y) const {
  Block* result = getIntersection(x, y);
  return result != nullptr;
}

Block* Simulation::getIntersection(float x, float y) const {
  for (int i = 0; i < interSections.size(); i++)
    if (interSections[i]->containsPoint(x, y)) return interSections[i];
  return nullptr;
}

vector<Vector2f> Simulation::getIntersectionCenter() {
  vector<Vector2f> IntersectionCenter;
  for (const auto& it : interSections)
    IntersectionCenter.push_back(it->getCenter());
  return IntersectionCenter;
}
