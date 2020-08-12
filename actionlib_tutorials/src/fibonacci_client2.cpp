#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "test_fibonacci");

  //create the action client
  // "fibonacci" is the server this client will call to 
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci",true);

  ROS_INFO("Waiting for action server start.");
  // Wait for the action server to start
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // Send a goal to the action 
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  // Wait for the action to return. Timeout is 30 seconds
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if( finished_before_timeout )
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}

