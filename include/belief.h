#ifndef BELIEF_H
#define BELIEF_H

/*
 * class Belief (Not Used)
 */
class Belief {
public:
  Belief(int _numElems, float value = -1) : numElems(_numElems) {
    if (value == -1) value = (1.0 / numElems);
    grid.resize(numElems);
    std::fill(grid.begin(), grid.end(), value);
  }

  float operator[](int i) { return grid[i]; }

  void setProb(int row, float p) { grid[row] = p; }

  void addProb(int row, float delta) {
    grid[row] += delta;
    assert(grid[row] >= 0.0);
  }

  // Returns the belief for tile row, col.
  float getProb(int row) { return grid[row]; }

  // Function: Normalize
  void normalize() {
    float total = getSum();
    for (int i = 0; i < numElems; i++) grid[i] /= total;
  }

  int getNumElems() { return numElems; }

  float getSum() {
    float total = 0.0;
    for (int i = 0; i < numElems; i++) total += getProb(i);
    return total;
  }

private:
  vector<float> grid;
  int numElems;
};

#endif /* BELIEF_H */
