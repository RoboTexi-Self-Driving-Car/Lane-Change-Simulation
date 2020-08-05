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
  vector<int> finishdata = layout.getFinish();
  finish = new Block(finishdata);
  host = new Host(Vector2f(startX, startY), startDir, Vector2f(0.0, 0.0));
  cars.push_back(host);

  for (vector<int> other : layout.getOtherData()) {
    Car* othercar =
        new Agent(Vector2f(other[0], other[1]), "east", Vector2f(0.0, 0.0));
    other_cars.push_back(othercar);
    cars.push_back(othercar);
  }

  car2index = UMAP<size_t, int>();

  for (int i = 0; i < other_cars.size(); i++) {
    car2index.insert({(size_t)other_cars[i], i});
  }
}

Simulation::Simulation(const Simulation& simulation) : layout(simulation.layout) {
  finish = simulation.finish;
  blocks = simulation.blocks;
  lines = simulation.lines;
  interSections = simulation.interSections;
  agentGraph = simulation.agentGraph;
  hostGraph = simulation.hostGraph;
  allGraph = simulation.allGraph;
  host = new Host(*simulation.getHost());
  host->setup();
  cars.push_back(host);

  for (Car* car : simulation.getOtherCars()) {
    Car* othercar = new Agent(*car);
    othercar->setup();
    other_cars.push_back(othercar);
    cars.push_back(othercar);
  }
}

Simulation::~Simulation() {
  if (cars.size() != 0) {
    while (cars.size() != 0) {
      delete cars.back();
      cars.pop_back();
    }
  }
}

void Simulation::setHost(Car* car) {
  vector<Car*> cars;
  cars.push_back(car);
  for (auto c : cars) {
    if (typeid(*c) != typeid(*host)) cars.push_back(c);
  }
  this->cars = cars;
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
    if (finish->containsPoint(point[0], point[1])) return true;
  }

  return false;
}

bool Simulation::checkCollision(Car* car) const {
  vector<Vector2f> bounds = car->getBounds();
  for (Vector2f point : bounds) {
    if (!inBounds(point.x, point.y)) return true;
  }

  for (Car* othercar : cars) {
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
