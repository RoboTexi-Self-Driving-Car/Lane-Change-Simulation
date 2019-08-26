#include "inference.h"

using namespace Inference;

int main(int argv, char argc) {
  vector<string> states = {"cooperative", "aggressive"};
  vector<vector<string>> output = permute(states);
  cout << "size of output: " << output.size() << endl;
  for(int i = 0; i < output.size(); i++) {
    for(int j = 0; j < output[0].size(); j++) {
      cout << output[i][j] << ", ";
    }
    cout << endl;
  }
  return 0;
}
