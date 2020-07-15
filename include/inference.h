#ifndef INFERENCE_H
#define INFERENCE_H

#include "model.h"
#include "car.h"

class Model;
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

  void initializeUniformly(const Model& model, const vector<string>& intentions);

  void initializeParticles();

  void observe(const Model& model);

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
  MarginalInference(int index, const Model& model);
  void initializeUniformly(const Model& gameState);
  void observe(const Model& gameState);
  vector<float> getBelief();

private:
  vector<string> legalIntentions;
  int index;
};

}  // namespace Inference

#endif
