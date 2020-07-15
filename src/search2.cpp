#include "search2.h"

namespace SEARCH2 {

//******************************************************************************
// State member functions
//******************************************************************************

State& State::operator=(const State& s) {
  current = s.current;
  actions = s.actions;
  cost = s.cost;
  heu = s.heu;
  return *this;
}

// State& State::operator=(State&& s) {
//        current = std::move(s.current);
//    actions = std::move(actions);
//        cost = std::move(s.cost);
//        heu = std::move(s.heu);
//    return *this;
// }

ostream& operator<<(ostream& os, const State& s) {
  os << "{current:[pos:" << s.current.first << ", dir" << s.current.second
     << "]"
     << ", pre:[action:" << s.actions.size() << "], cost:" << s.cost
     << ",heu: " << s.heu << "}" << std::endl;
  return os;
}

//******************************************************************************
// Search member functions
//******************************************************************************

Search::Search(Model* m, const vec2f& goal) : model(m) {
  vec2f pos = model->getHost()->getPos();
  start = State(pvff(pos, vec2f(1, 0)));
  this->goal = goal;
  cost = 1;
  unitdistanace = 10;

  for (float ang = 45; ang >= -45; ang -= 15) angle.push_back(ang);

  State state2 = search();
  list<char> actions = state2.actions;
  pa = path(actions);
}

bool Search::isGoal(State& s) {
  float x = s.current.first[0];
  float y = s.current.first[1];

  if (abs(x - goal[0]) < unitdistanace && abs(y - goal[1]) < unitdistanace)
    return true;

  return false;
}

vector<vec2f> Search::path(list<char>& actions) {
  vector<vec2f> result;
  //    vec2f pos = start.current.first;
  //    vec2f olddir = start.current.second;
  //    vec2f velocity = olddir*float(unitdistanace);
  //    Car car(pos, olddir, velocity);
  State state = start;
  vec2f pos = state.current.first;
  vec2f olddir = state.current.second;
  vec2f velocity = olddir * float(unitdistanace);
  result.push_back(pos);

  for (const auto& c : actions) {
    Car car(pos, olddir, velocity);
    car.setWheelAngle(angle[c - 'A']);
    car.update();
    pos = car.getPos();
    olddir = car.getDir();
    velocity = olddir * float(unitdistanace);

    float x = car.getPos()[0];
    float y = car.getPos()[1];
    //        int col = xToCol(x);
    //        int row = yToRow(y);
    vec2f newpos(x, y);
    result.push_back(newpos);
  }

  return result;
}

vector<State> Search::getSuccessors(const State& state) {
  /*
  Returns successor states, the actions they require, and a cost of 1.
  As noted in search.py:
  For a given state, this should return a list of triples,
  (successor, action, stepCost), where 'successor' is a
  */
  vector<State> successors;
  //    float ScaleRatio = 1.5;
  vec2f pos = state.current.first;
  vec2f olddir = state.current.second;
  vec2f velocity = olddir * float(unitdistanace);

  for (int i = 0; i < num_action(); i++) {
    list<char> actions = state.actions;
    float oldcost = state.cost;
    Car car(pos, olddir, velocity);
    car.setWheelAngle(angle[i]);
    car.update();
    // before it was 1.5* car::length now i change to 1 to suit 'road' case
    vector<vec2f> bounds =
        car.getBounds(car, 1.2 * Car::LENGTH, 1.2 * Car::WIDTH);
    bool isinBound = true;

    for (const auto& point : bounds) {
      if (!model->inBoundsLarger(point[0], point[1])) {
        isinBound = false;
        break;
      }
    }

    //        vector<Car*> cars = model->getOtherCars();
    //        for (Car* othercar : cars) {
    //            if (othercar->collides(car.getPos(), bounds)){
    //                isinBound = false;
    //                break;
    //            }
    //        }

    if (!isinBound) continue;

    vec2f newPos = car.getPos();
    vec2f newdir = car.getDir();
    // I remove the 400 item here for to ajust for the second to suit 'road'
    // case
    oldcost = oldcost + cost + 10 * abs(angle[i]) / 180;
    // now the new ones is as like this
    // oldcost = oldcost + cost  + 400*abs(angle[i])/180;
    actions.push_back(char(i + 'A'));
    successors.push_back(State({newPos, newdir}, actions, oldcost));
  }

  return successors;
}

State Search::search() {
  priority_queue<State> open;
  unordered_set<pii> closed;
  float c, h;
  h = manhattanHeuristic(start.current.first);
  start.heu = h;
  open.push(start);

  // int row,col;
  pii cell;
  while (!open.empty()) {
    State state = open.top();
    open.pop();
    vec2f position = state.current.first;
    vec2f dir = state.current.second;
    // int action = int (state.actions.back()-'A');
    c = state.cost;
    h = state.heu;

    if (isGoal(state)) return state;

    cell = pii(yToRow(position[1]), xToCol(position[0]));
    if (closed.count(cell) != 0) continue;
    closed.insert(cell);
    // get sucesssor
    vector<State> successors = getSuccessors(state);

    if (successors.size() == 0) continue;
    for (auto& ele : successors) {
      position = ele.current.first;
      cell = pii(yToRow(position[1]), xToCol(position[0]));
      if (closed.count(cell) == 0) {
        h = evaluation(position);
        ele.heu = h;
        // State state2(ele.current, ele.actions, cost, h);
        open.push(ele);
      }
    }
  }

  State state2;

  return state2;
}
// smooth the path a little bit
void Search::smooth() {
  float tolerance = 0.000001;
  float weight_data = 0.1;
  float weight_smooth = 0.1;

  vector<vec2f> result(pa);
  float change = 1;

  while (change > tolerance) {
    change = 0.0;
    for (int i = 1; i < pa.size() - 1; i++) {
      for (int j = 0; j < 2; j++) {
        float aux = result[i][j];
        result[i][j] += weight_data * (pa[i][j] - result[i][j]);
        result[i][j] += weight_smooth * (result[i - 1][j] + result[i + 1][j] -
                                         2 * result[i][j]);
        if (i >= 2)
          result[i][j] +=
              0.5 * weight_smooth *
              (2.0 * result[i - 1][j] - result[i - 2][j] - result[i][j]);
        if (i <= pa.size() - 3)
          result[i][j] +=
              0.5 * weight_smooth *
              (2.0 * result[i + 1][j] - result[i + 2][j] - result[i][j]);
        change += abs(aux - result[i][j]);
      }
    }
  }
  pa = result;
}

// evaluate the path
float Search::evaluation(const vec2f& position) {
  float h = manhattanHeuristic(position);
  return h;
}

}
