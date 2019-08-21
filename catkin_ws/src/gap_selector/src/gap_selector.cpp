#include <vector>

#include "ros/ros.h"
#include "traffic_msgs/VehicleState.h"
#include "traffic_msgs/VehicleStateArray.h"
#include "traffic_msgs/Gap.h"

using namespace std;


void trafficInfoCallback(const traffic_msgs::VehicleStateArray::ConstPtr& msg) {
  vector<traffic_msgs::VehicleState> vehicles = msg->vehicles;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gap_selector");
  ros::NodeHandle n;

  ros::Publisher selected_gap_pub = n.advertise<traffic_msgs::Gap>("/selected_gap", 1000);

  ros::Rate rate(10);

  while(ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
