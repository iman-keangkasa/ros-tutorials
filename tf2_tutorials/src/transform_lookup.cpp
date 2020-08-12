#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_lookup");
  ros::NodeHandle node;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while (node.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      // ros::Time(0) will invoke the lookupTransform for the latest transform
      //transformStamped = tfBuffer.lookupTransform("moving_frame", "map",ros::Time(0));

      //ros::Time::now() will invoke warning and lookupTransform will not get the 
      //transform I need; add a timeout. However, timeout is a blocking.
      transformStamped = tfBuffer.lookupTransform("map", "moving_frame",ros::Time::now(), ros::Duration(1));
      ROS_INFO("%s",transformStamped.header.frame_id.c_str());
      ROS_INFO("Affine: [%5.3f %5.3f %5.3f]", 
          transformStamped.transform.translation.x, 
          transformStamped.transform.translation.y,
          transformStamped.transform.translation.z);
      ROS_INFO("Rotation:[%5.3f %5.3f %5.3f | %5.3f]", 
          transformStamped.transform.rotation.x,
          transformStamped.transform.rotation.y,
          transformStamped.transform.rotation.z,
          transformStamped.transform.rotation.w);
 
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
  return 0;
}
