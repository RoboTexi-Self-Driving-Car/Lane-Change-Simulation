
/*
 * This file is for any helper function not listed in other files, will be here
 */
#ifndef UTIL_H
#define UTIL_H

extern GLFWwindow* window;
extern Display display;

// initiation to start the open gl program
void begin_graphics(int SCREEN_WIDTH, int SCREEN_HEIGHT, string title) {
  if (!glfwInit()) {
    return;
  }

  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

  // create a window mode window and its openGL context
  window =
      glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, title.c_str(), nullptr, nullptr);

  if (!window) {
    glfwTerminate();
    return;
  }

  glfwMakeContextCurrent(window);
  glOrtho(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT, 0, 1);
}

// to check if the game is over
bool gameover(Simulation& simulation) {
  if (simulation.checkVictory() || simulation.checkCollision(simulation.getHost()))
    return true;
  return false;
}

// draw lines
void drawPolygon(vector<Vector2f>& polygonvertices) {
  glPushAttrib(GL_POLYGON_BIT);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glColor3f(1.0f, 0.0f, 0.0f);
  float* vertices = new float[polygonvertices.size() * 2];
  glLineWidth(2);
  glBegin(GL_LINE_STRIP);

  for (int i = 0; i < polygonvertices.size(); i++) {
    vertices[2 * i] = polygonvertices[i][0];
    vertices[2 * i + 1] = polygonvertices[i][1];
    glVertex2d(vertices[2 * i], vertices[2 * i + 1]);
  }

  glEnd();
  glPopAttrib();
  // glEnableClientState(GL_VERTEX_ARRAY);
  // glVertexPointer(2, GL_FLOAT, 0, vertices);
  // glDrawArrays(GL_POLYGON, 0, polygonvertices.size());
  // glDisableClientState(GL_VERTEX_ARRAY);
  delete[] vertices;
}

void observe(Host* ego, const Simulation& simulation) {
  ego->makeObservation(simulation);
  vector<Actor*> cars = simulation.getOtherCars();
  for (int i = 0; i < cars.size(); i++) {
    Car* car = dynamic_cast<Car*>(cars[i]);
    int index = simulation.getIndex(car);
    Inference::MarginalInference* inference = car->getInference(index + 1, simulation);
  }
}

vector<int> infer(const Simulation& simulation) {
  Host* ego = dynamic_cast<Host*>(simulation.getHost());
  vector<int> car2intention;
  observe(ego, simulation);

  // beliefs = []
  vector<string> colors{"orange", "red"};
  vector<Actor*> cars = simulation.getOtherCars();
  for (int i = 0; i < cars.size(); i++) {
    Car* car = dynamic_cast<Car*>(cars[i]);
    int index = simulation.getIndex(car);
    vector<float> belief = car->getInference(index + 1, simulation)->getBelief();

    int max_index = 0;
    for (int j = 0; j < belief.size(); j++) {
      if (belief[j] > belief[max_index]) {
        max_index = j;
      }
    }

    car2intention.push_back(max_index);
    display.colorChange(car, colors[max_index]);
  }

  return car2intention;
}

#endif /* UTIL_H */
