#include <iostream>
#include <vector>
#include <limits>

#include "ros/ros.h"
#include "traffic_msgs/VehicleState.h"
#include "traffic_msgs/VehicleStateArray.h"
#include "traffic_msgs/Gap.h"

using namespace std;

traffic_msgs::VehicleState ego;
traffic_msgs::Gap selected_gap;

void egoStateCallback(const traffic_msgs::VehicleState::ConstPtr& msg) {
  cout << "ego state got ..." << endl;
  ego.pose.pose = msg->pose.pose;
  ego.twist.twist = msg->twist.twist;
}

void trafficStateCallback(const traffic_msgs::VehicleStateArray::ConstPtr& msg) {
  vector<traffic_msgs::VehicleState> vehicles = msg->vehicles;

  cout << "+++++ Vehicles in the target lane: +++++" << endl;
  for (traffic_msgs::VehicleState veh : vehicles) {
    //other car moves
    cout << "  ------------" << endl;
    cout << "  position: " << veh.pose.pose.position.x << ", " << veh.pose.pose.position.y << endl;
    cout << "  velocity: " << veh.twist.twist.linear.x << ", " << veh.twist.twist.linear.y << endl;
  }

  double distance_to_ego = numeric_limits<double>::max();

  cout << "+++++ Gaps in the target lane: +++++" << endl;
  for (int i = 0; i < vehicles.size()-1; i++) {
    traffic_msgs::VehicleState leader = vehicles[i];
    traffic_msgs::VehicleState follower = vehicles[i+1];
    cout << "  ------------" << endl;
    double gap_size = leader.pose.pose.position.x - follower.pose.pose.position.x;
    cout << "  gap size: " << gap_size << endl;
    double gap_x = (leader.pose.pose.position.x + follower.pose.pose.position.x) / 2;
    double gap_y = (leader.pose.pose.position.y + follower.pose.pose.position.y) / 2;
    cout << "  gap center: " << gap_x << ", " << gap_y << endl;
    // double ego_x = ego.pose.pose.position.x;
    // if(gap_x < ego_x && ego_x - gap_x < distance_to_ego) {
    //   cout << "  ## update selected_gap: ##" << endl;
    //   distance_to_ego = ego_x - gap_x;
    //   selected_gap.center.x = gap_x;
    //   selected_gap.center.y = gap_y;
    //   selected_gap.width = 5.0; // to be modified
    //   selected_gap.length = gap_size;
    //   selected_gap.leader = leader;
    //   selected_gap.follower = follower;
    // }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gap_selector");
  ros::NodeHandle n;

  ros::Subscriber ego_state_sub = n.subscribe("/ego_state", 1000, egoStateCallback);
  ros::Subscriber vehicle_array_sub = n.subscribe("/vehicles_in_target_lane", 1000, trafficStateCallback);

  ros::Publisher selected_gap_pub = n.advertise<traffic_msgs::Gap>("/selected_gap", 1000);

  ros::Rate rate(10);

  while(ros::ok()) {
    selected_gap_pub.publish(selected_gap);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
