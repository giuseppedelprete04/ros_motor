#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ros_motor/motorAction.h"
#include <iostream>

int main (int argc, char** argv) {
  ros::init(argc, argv, "motor_client");
  if(argc!=5) {
    ROS_INFO("%d",argc);
    ROS_WARN("Usage: motor_client <init_pos> <des_pos> <max_speed> <time>");
    return 1;
  }

  actionlib::SimpleActionClient<ros_motor::motorAction> ac("motor_server",true);
  ROS_INFO("Waiting for server to start.");
  ac.waitForServer();
  ROS_INFO("Server started");

  ros_motor::motorGoal goal;
  goal.init_pos = atoi(argv[1]);
  goal.des_pos = atoi(argv[2]);
  goal.max_speed = atoi(argv[3]);
  goal.time = atoi(argv[4]);

  ROS_INFO("Sending Goal");
  ac.sendGoal(goal);
  bool finished_before_timeout = ac.waitForResult (ros::Duration(atoi(argv[4])+2));
  ac.cancelGoal();

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    ac.cancelGoal();
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
