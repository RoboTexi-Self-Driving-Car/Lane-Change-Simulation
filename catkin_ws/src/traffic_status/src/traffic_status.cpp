////
////  main.cpp
////  TestOpenGL
////
////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
////
//
#include <iostream>
#include "picojson.h"
#include "layout.h"
#include "display.h"
#include "globals.h"
#include "model.h"
#include "search.h"
#include "inference.h"
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <iomanip>
#include <fstream>

#include "KdTree.hpp"
#include "decisionmaking2.h"
#include "util.h"

using namespace std;


GLFWwindow *window = NULL;

Display display = Display();
ofstream myfile;

int main(void) {

  myfile.open ("intention.txt"); // ?

  // load the map for running testing examples
  string worldname = "dense_traffic";
  Layout layout = Layout(worldname);
  Model model(layout);

  display.setColors(model.getCars());

  Car* mycar = model.getHost();
  std::cout << "***** Check host: *****" << std::endl;
  std::cout << mycar->isHost() << std::endl;
  std::cout << "***** Check mycar type: *****" << std::endl;
  std::cout << typeid(mycar).name() << std::endl;
  std::cout << std::endl;

  vector<Car*> cars = model.getCars();

  int SCREEN_WIDTH = layout.getWidth();
  int SCREEN_HEIGHT = layout.getHeight();
  string title = Globals::constant.TITLE;
  begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);

  // loop util the user closes the window
  // bool gameover = false;

  bool over = false;
  DecisionAgent2 decision;
  vector<vec2f> mypath;
  vector<int> carintentions;

  for (int i = 0; i < model.getOtherCars().size(); i++) {
    carintentions.push_back(1);
  }

  bool success = decision.getPath(model, mypath, carintentions);

  // debug: check mypath
  std::cout << "***** Check mypath: *****" << std::endl;
  for(int i = 0; i < mypath.size(); i++) {
    std::cout << mypath[i].x << ", " << mypath[i].y << std::endl;
  }
  std::cout << std::endl;

  vector<vector<vec2f>> mypaths = decision.getPaths();
  // debug: check mypaths
  std::cout << "***** Check mypaths: *****" << std::endl;
  for(int i = 0; i < mypaths.size(); i++) {
    std::cout << "path " << i << std::endl;
    for(int j = 0; j < mypaths[0].size(); j++) {
      std::cout << mypaths[i][j].x << ", " << mypaths[i][j].y << std::endl;
    }
  }
  std::cout << std::endl;

  //std::pair<std::string, vec2f> actionset; // ?

  //string filename = "coop"; // ?

  // the change for car is mandatory
  bool change; // = true;
  srand(time(NULL));
  //int i = 0;

  while(!glfwWindowShouldClose(window)) {
    glClearColor(1.0f, 1.0f, 1.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Display::drawGoal(model.getFinish());
    Display::drawBlocks(model.getBlocks());
    Display::drawLine(model.getLine());

    // for(auto p : mypaths) {
    //   drawPolygon(p);
    // }
    for(int i = 0; i < mypaths.size(); i++) {
      drawPolygon(mypaths[i]);
    }

    display.drawCar(model.getHost());
    display.drawOtherCar(model.getOtherCars());

    if (!gameover(model)) {
      //update cars here for further processing
      for (Car* car: cars) {
        if (car == mycar) { //my car moves
          //destination reaches, generate new paths
          if (mypath.size() == 0 || abs(mycar->getPos().x - mypath[mypath.size()-1].x) < 10) {
            success = decision.getPath(model, mypath, carintentions);
            mypaths = decision.getPaths();
          }
          car->autonomousAction(mypath, model, NULL);
          car->update();
          drawPolygon(mypath);
        } else { //other car moves
          car->autonomousAction(mypath, model, NULL);
          car->update();
        }
      }
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
    over = gameover(model) || glfwWindowShouldClose(window);
    Display::sleep(0.05);
  }

  if (model.checkVictory())
  std::cout << "The car win" << endl;
  else
  std::cout << "You lose the car game" << endl;

  glfwTerminate();
  myfile.close();
  return 1;
}
