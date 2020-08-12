#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

void movingFrameCb(const ros::TimerEvent&)
{
  // Static declares so that it would 
  // be create each time a callback is invoked
  static uint32_t counter = 0;
  static tf2_ros::TransformBroadcaster br;
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "moving_frame";
  transformStamped.child_frame_id = "carrot1";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
  counter++;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "adding_frames");
  ros::NodeHandle nh;
  ros::Timer frame_time = nh.createTimer(ros::Duration(0.01), movingFrameCb);  
  ros::spin();
}
