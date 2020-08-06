#ifndef INFERENCE_H
#define INFERENCE_H

#include "simulation.h"
#include "car.h"

class Simulation;
class Actor;

namespace Inference {

enum Intention {cooperative, aggressive};

const vector<string> g_intentions{"cooperative", "aggressive"};

const UMAP<string, int> g_intention2index{{"cooperative", 0}, {"aggressive", 1}};

//************************************************************************
// class JointParticles
//************************************************************************

class JointParticles {
public:
  JointParticles(int num = 600) : num_particles(num), num_cars(0) {};

  void initializeUniformly(const Simulation& simulation, const vector<string>& intentions);

  void initializeParticles();

  void observe(const Simulation& simulation);

  Counter<vector<string>> getBelief();

  vector<string> sample(Counter<vector<string>>& distribution);

  pff getMeanStandard(queue<float>& history, const string& intention);

private:
  int num_particles;
  int num_cars;
  vector<string> legal_intentions;
  vector<Actor*> cars;
  Counter<vector<string>> beliefs;
  vector<vector<string>> particles;
};

static JointParticles jointInference = JointParticles();

//************************************************************************
// class MarginalInference
//************************************************************************

class MarginalInference {
public:
  MarginalInference(int index, const Simulation& simulation);
  void initializeUniformly(const Simulation& simulation);
  void observe(const Simulation& simulation);
  vector<float> getBelief();

private:
  vector<string> legal_intentions;
  int index;
};

}  // namespace Inference

#endif
