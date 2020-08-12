#include <ros/ros.h>
#include <service_tutorials/AddTwoInts.h>
#include <geometry_msgs/PoseStamped.h>

bool addCb(service_tutorials::AddTwoInts::Request &req,
           service_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("Request: x=%ld, y=%ld", req.a, req.b);
  ROS_INFO("Response: %ld", res.sum);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints",addCb);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
