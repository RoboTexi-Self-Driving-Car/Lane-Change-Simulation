////
////  main.cpp
////  TestOpenGL
////
////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
////
//
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "decision_making.h"
#include "display.h"
#include "inference.h"
#include "util.h"

using namespace std;

GLFWwindow* window = nullptr;

Display display = Display();
ofstream myfile;

int main(void) {
  //****************************************************************************
  // Load the map.
  //****************************************************************************
  myfile.open("intention.txt");

  string worldname = "road2";
  Layout layout = Layout(worldname);
  Simulation simulation(layout);

  // Display setting.
  display.setColors(simulation.getAllCars());

  int SCREEN_WIDTH = layout.getWidth();
  int SCREEN_HEIGHT = layout.getHeight();

  // Get the ego car.
  Actor* ego_car = simulation.getHost();

  // Get the neighboring cars.
  vector<Actor*> cars = simulation.getAllCars();

  // Display setting.
  string title = Globals::constant.TITLE;
  begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);
  std::cout << "[Simulation]: " << typeid(*ego_car).name() << std::endl;

  // loop util the user closes the window
  // bool gameover = false;
  bool over = false;

  // decision making module
  DecisionMaker decision;

  // final path
  vector<Vec2f> final_path;

  // each neighboring cars' yielding intention
  vector<int> car_intentions;
  for (int i = 0; i < simulation.getOtherCars().size(); i++) {
    car_intentions.push_back(1);
  }

  bool success = decision.getPath(simulation, final_path, car_intentions);

  // get candidate paths
  vector<vector<Vec2f>> candidate_paths = decision.getPaths();

  // the change for car is mandatory
  bool change = true;

  while (!glfwWindowShouldClose(window)) {
    //**************************************************************************
    // Display static taffic objects.
    //**************************************************************************
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Display::drawGoal(simulation.getGoal());
    Display::drawBlocks(simulation.getBlocks());
    Display::drawLine(simulation.getLine());

    //**************************************************************************
    // Draw candidate paths.
    //**************************************************************************
    for (auto p : candidate_paths) {
      drawPolygon(p);
    }

    //**************************************************************************
    // Display the cars.
    //**************************************************************************
    display.drawCar(simulation.getHost());
    display.drawOtherCar(simulation.getOtherCars());

    if (!gameover(simulation)) {
      //************************************************************************
      // Update all the cars in a time step
      //************************************************************************
      for (Actor* car : cars) {
        // my car moves
        if (car == ego_car) {
          // destination reaches, generate new paths
          if (final_path.size() == 0 || abs(ego_car->getPos().x - final_path[final_path.size() - 1].x) < 10) {
            success = decision.getPath(simulation, final_path, car_intentions);
            change = decision.isChangeRequired(car, simulation);
            // candidate paths
            candidate_paths = decision.getPaths();
            if (!success && change) {
              car_intentions = infer(simulation);
              final_path.clear();
              decision.applyAction(simulation, 0, "dec");
              car->update();
            } else {
              car->autonomousAction(final_path, simulation, nullptr);
              car->update();
            }
          }
          // using the current path
          else {
            car->autonomousAction(final_path, simulation, nullptr);
            car->update();
          }
          // display the final path
          drawPolygon(final_path);
        }
        // other car moves
        else {
          car->autonomousAction(final_path, simulation, nullptr);
          car->update();
        }
      }
    }

    //************************************************************************
    // Update the display window.
    //************************************************************************
    glfwSwapBuffers(window);
    glfwPollEvents();

    //************************************************************************
    // Check the ending contion.
    //************************************************************************
    over = (gameover(simulation) || glfwWindowShouldClose(window));

    Display::sleep(0.05);
  }

  if (simulation.checkVictory()) {
    std::cout << "[Simulation]: The car win." << endl;
  }
  else {
    std::cout << "[Simulation]: You lose the car game." << endl;
  }

  //************************************************************************
  // Close the simulation.
  //************************************************************************
  glfwTerminate();
  myfile.close();

  return 0;
}
