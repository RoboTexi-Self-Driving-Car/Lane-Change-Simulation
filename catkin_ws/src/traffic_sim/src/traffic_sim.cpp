////
////  main.cpp
////  TestOpenGL
////
////  Created by HJKBD on 8/20/16.
////  Copyright © 2016 HJKBD. All rights reserved.
////
//
#include "picojson.h"
#include "layout.h"
#include "display.h"
#include "globals.h"
#include "model.h"
#include "search.h"
#include "inference.h"
#include "KdTree.hpp"
#include "decisionmaking2.h"
#include "util.h"

#include "ros/ros.h"
#include "traffic_msgs/VehicleState.h"
#include "traffic_msgs/VehicleStateArray.h"

#include <iostream>
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <iomanip>
#include <fstream>

using namespace std;


GLFWwindow *window = NULL;

Display display = Display();
// ofstream myfile;

traffic_msgs::VehicleState ego_state;

traffic_msgs::VehicleStateArray update_traffic_info(vector<Car*> cars) {
  traffic_msgs::VehicleStateArray veh_array;

  Car* host;
  vector<Car*> cars_in_target_lane;
  for (Car* car: cars) {
    if (car->isHost()) { //my car moves
      cout << "***** Ego Car: *****" << endl;
      cout << "  position: " << car->getPos().x << ", " << car->getPos().y << endl;
      cout << "  velocity: " << car->getVelocity().x << ", " << car->getVelocity().y << endl;
      cout << "  dir: " << car->getDir().x << ", " << car->getDir().y << endl;
      host = car;
      ego_state.pose.pose.position.x = car->getPos().x;
      ego_state.pose.pose.position.y = car->getPos().y;
      ego_state.pose.pose.position.z = 0;
      ego_state.twist.twist.linear.x = car->getVelocity().x;
      ego_state.twist.twist.linear.y = car->getVelocity().y;
    }
  }

  for (Car* car: cars) {
    if (!car->isHost()) {
      if ( abs(car->getPos().y - host->getPos().y) > 0.5 ) {
        cars_in_target_lane.push_back(car);
      }
    }
  }

  sort(cars_in_target_lane.begin(), cars_in_target_lane.end(),
       [](Car* a, Car* b) { return a->getPos().x > b->getPos().x; });

  cout << "+++++ Cars in the target lane: +++++" << endl;
  for (Car* car: cars_in_target_lane) {
    //other car moves
    cout << "  ------------" << endl;
    cout << "  position: " << car->getPos().x << ", " << car->getPos().y << endl;
    cout << "  velocity: " << car->getVelocity().x << ", " << car->getVelocity().y << endl;
    cout << "  dir: " << car->getDir().x << ", " << car->getDir().y << endl;

    traffic_msgs::VehicleState veh;
    veh.pose.pose.position.x = car->getPos().x;
    veh.pose.pose.position.y = car->getPos().y;
    veh.pose.pose.position.z = 0;
    veh.twist.twist.linear.x = car->getVelocity().x;
    veh.twist.twist.linear.y = car->getVelocity().y;
    veh_array.vehicles.push_back(veh);
  }

  cout << "+++++ Gaps in the target lane: +++++" << endl;
  for (int i = 0; i < cars_in_target_lane.size()-1; i++) {
    Car* leader = cars_in_target_lane[i];
    Car* follower = cars_in_target_lane[i+1];
    //other car moves
    cout << "  ------------" << endl;
    cout << "  gap size: " << leader->getPos().x - follower->getPos().x << endl;
    double gap_x = (leader->getPos().x + follower->getPos().x) / 2;
    double gap_y = (leader->getPos().y + follower->getPos().y) / 2;
    cout << "  gap center: " << gap_x << ", " << gap_y << endl;
    cout << "  distance to ego: " << gap_x - host->getPos().x << endl;
  }

  return veh_array;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "traffic_sim");
  ros::NodeHandle n;

  ros::Publisher ego_state_pub = n.advertise<traffic_msgs::VehicleState>("/ego_state", 1000);
  ros::Publisher veh_array_pub = n.advertise<traffic_msgs::VehicleStateArray>("/vehicles_in_target_lane", 1000);

  ros::Rate rate(10);

  // load the map for running testing examples
  string worldname = "dense_traffic";
  Layout layout(worldname);
  Model model(layout);

  display.setColors(model.getCars());

  Car* mycar = model.getHost();
  // cout << "***** Check host: *****" << endl;
  // cout << mycar->isHost() << endl;
  // cout << "***** Check mycar type: *****" << endl;
  // cout << typeid(mycar).name() << endl;
  // cout << endl;

  vector<Car*> cars = model.getCars();

  int SCREEN_WIDTH = layout.getWidth();
  int SCREEN_HEIGHT = layout.getHeight();
  string title = Globals::constant.TITLE;

  // Initlize the simulation window.
  begin_graphics(SCREEN_WIDTH, SCREEN_HEIGHT, title);

  bool over = false;

  DecisionAgent2 decision;
  vector<vec2f> mypath;
  vector<int> carintentions;

  for (int i = 0; i < model.getOtherCars().size(); i++) {
    carintentions.push_back(1);
  }

  bool success = decision.getPath(model, mypath, carintentions);
  vector<vector<vec2f>> mypaths = decision.getPaths();

  bool change;
  srand(time(NULL));

  while(ros::ok() && !glfwWindowShouldClose(window)) {
    glClearColor(1.0f, 1.0f, 1.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    Display::drawGoal(model.getFinish());
    Display::drawBlocks(model.getBlocks());
    Display::drawLine(model.getLine());

    for(int i = 0; i < mypaths.size(); i++) {
      drawPolygon(mypaths[i]);
    }

    display.drawCar(model.getHost());
    display.drawOtherCar(model.getOtherCars());

    if (!gameover(model)) {
      // public ego state and the cars in the target lane
      traffic_msgs::VehicleStateArray veh_array = update_traffic_info(cars);
      ego_state_pub.publish(ego_state);
      veh_array_pub.publish(veh_array);

      // update cars here for further processing
      for (Car* car: cars) {
        if (car == mycar) { //my car moves
          // //destination reaches, generate new paths
          // if (mypath.size() == 0 || abs(mycar->getPos().x - mypath[mypath.size()-1].x) < 10) {
          //   success = decision.getPath(model, mypath, carintentions);
          //   mypaths = decision.getPaths();
          // }
          // generate paths in each time step
          success = decision.getPath(model, mypath, carintentions);
          mypaths = decision.getPaths();
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
    Display::sleep(0.1);
  }

  if (model.checkVictory()) {
    cout << "The car win" << endl;
  } else {
    cout << "You lose the car game" << endl;
  }

  glfwTerminate();

  return 1;
}
