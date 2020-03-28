#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "ros_motor/motorAction.h"
#include <iostream>
#include <sstream>

class action_class {
  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ros_motor::motorAction> as;
    ros_motor::motorFeedback feedback;
    ros_motor::motorResult result;

    std::string action_name;
    int goal;
    float progress;

  public:
    action_class(std::string name) : as(nh_, name, boost::bind(&action_class::executeCB, this, _1), false), action_name(name) {
      as.registerPreemptCallback(boost::bind(&action_class::preemptCB,this));
      as.start();
    }

  void preemptCB () {
    ROS_WARN("%s got preempted!", action_name.c_str());
    result.final_pos = progress;
    as.setPreempted(result, "I got Preempted");
  }

  void executeCB (const ros_motor::motorGoalConstPtr &goal) {
    if(!as.isActive() || as.isPreemptRequested() ) return;

    float frequency = 10;
    ros::Rate rate(5);

    ROS_INFO("%s is processing the goal: init_pos=%d, des_pos=%d, max_speed=%d, max_time=%d", action_name.c_str(),goal->init_pos, goal->des_pos,goal->max_speed,goal->time);
    float speed = (float)(goal->des_pos - goal->init_pos)/goal->time;
    if (speed > goal->max_speed) speed = goal->max_speed;
    for (progress = goal->init_pos; progress <= (goal->des_pos+1); progress=progress+(speed/frequency)) {
      if ( !ros::ok() ) {
        result.final_pos = progress;
        as.setAborted(result, "I failed!");
        ROS_INFO("%s Shutting down", action_name.c_str());
        break;
      }
      if (!as.isActive() || as.isPreemptRequested()) {
        return;
      }
      if (goal->des_pos <= progress) {
        ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->des_pos);
        result.final_pos = progress;
        as.setSucceeded(result);
      }
      else {
        ROS_INFO("Setting to goal %f / %d", feedback.curr_pos, goal->des_pos);
        feedback.curr_pos = progress;
        as.publishFeedback(feedback);
      }
      rate.sleep();
    }
  }

};

int main (int argc, char **argv) {
  ros::init(argc,argv,"motor_server");
  ROS_INFO("Starting Motor Server");
  action_class motor_action_obj(ros::this_node::getName());
  ros::spin();
  return 0;
}
