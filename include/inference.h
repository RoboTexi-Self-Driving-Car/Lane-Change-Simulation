#ifndef INFERENCE_H
#define INFERENCE_H

#include "simulation.h"
#include "car.h"

class Simulation;
class Car;

namespace Inference {

enum State {cooperative, aggressive};

const vector<string> intentions{"cooperative", "aggressive"};

const UMAP<string, int> Intention_To_Index{{"cooperative", 0}, {"aggressive", 1}};

//************************************************************************
// class JointParticles
//************************************************************************

class JointParticles {
public:
  JointParticles(int num = 600) : numParticles(num), numAgents(0) {};

  void initializeUniformly(const Simulation& simulation, const vector<string>& intentions);

  void initializeParticles();

  void observe(const Simulation& simulation);

  Counter<vector<string>> getBelief();

  vector<string> sample(Counter<vector<string>>& counter);

  pff getMeanStandard(queue<float>& history, const string& intention);

private:
  int numParticles;
  int numAgents;
  vector<string> legalIntentions;
  vector<Car*> agents;
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
  void initializeUniformly(const Simulation& gameState);
  void observe(const Simulation& gameState);
  vector<float> getBelief();

private:
  vector<string> legalIntentions;
  int index;
};

}  // namespace Inference

#endif
