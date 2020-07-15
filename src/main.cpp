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

#include "decision_making_2.h"
#include "display.h"
#include "inference.h"
#include "util.h"

using namespace std;

GLFWwindow* window = NULL;

Display display = Display();
ofstream myfile;

int main(void) {
  //****************************************************************************
  // Load the map.
  //****************************************************************************
  myfile.open("intention.txt");

  string worldname = "road2";
  Layout layout = Layout(worldname);
  Model model(layout);

  // Display setting.
  display.setColors(model.getCars());

  int SCREEN_WIDTH = layout.getWidth();
  int SCREEN_HEIGHT = layout.getHeight();

  // Get the ego car.
  Car* mycar = model.getHost();

  // Get the neighboring cars.
  vector<Car*> cars = model.getCars();
  // std::cout<<car->isHost()<<std::endl;

  // Display setting.
  string title = Globals::constant.TITLE;
  begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);
  std::cout << typeid(*mycar).name() << std::endl;

  // loop util the user closes the window
  // bool gameover = false;
  bool over = false;

  // decision making module
  DecisionAgent2 decision;

  // final path
  vector<vec2f> mypath;

  // each neighboring cars' yielding intention
  vector<int> carintentions;
  for (int i = 0; i < model.getOtherCars().size(); i++) {
    carintentions.push_back(1);
  }

  bool success = decision.getPath(model, mypath, carintentions);

  // get candidate paths
  vector<vector<vec2f>> mypaths = decision.getPaths();

  // action set
  std::pair<std::string, vec2f> actionset;

  string filename = "coop";

  // the change for car is mandatory
  bool change = true;

  srand(time(NULL));

  int i = 0;

  while (!glfwWindowShouldClose(window)) {
    //**************************************************************************
    // Display static taffic objects.
    //**************************************************************************
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Display::drawGoal(model.getFinish());
    Display::drawBlocks(model.getBlocks());
    Display::drawLine(model.getLine());

    //**************************************************************************
    // Draw candidate paths.
    //**************************************************************************
    for (auto p : mypaths) {
      drawPolygon(p);
    }

    //**************************************************************************
    // Display the cars.
    //**************************************************************************
    display.drawCar(model.getHost());
    display.drawOtherCar(model.getOtherCars());

    if (!gameover(model)) {
      //************************************************************************
      // Update all the cars in a time step
      //************************************************************************
      for (Car* car : cars) {
        // my car moves
        if (car == mycar) {
          // destination reaches, generate new paths
          if (mypath.size() == 0 || abs(mycar->getPos().x - mypath[mypath.size() - 1].x) < 10) {
            success = decision.getPath(model, mypath, carintentions);
            change = decision.isChangeRequired(car, model);
            // candidate paths
            mypaths = decision.getPaths();
            if (!success && change) {
              carintentions = infer(model);
              mypath.clear();
              decision.ApplyAction(model, 0, "dec");
            } else {
              car->autonomousAction(mypath, model, NULL);
              car->update();
            }
          }
          // using the current path
          else {
            car->autonomousAction(mypath, model, NULL);
            car->update();
          }
          // display the final path
          drawPolygon(mypath);
        }
        // other car moves
        else {
          car->autonomousAction(mypath, model, NULL);
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
    over = (gameover(model) || glfwWindowShouldClose(window));

    Display::sleep(0.05);
  }

  if (model.checkVictory()) {
    std::cout << "The car win" << endl;
  }
  else {
    std::cout << "You lose the car game" << endl;
  }

  //************************************************************************
  // Close the simulation.
  //************************************************************************
  glfwTerminate();
  myfile.close();

  return 0;
}
