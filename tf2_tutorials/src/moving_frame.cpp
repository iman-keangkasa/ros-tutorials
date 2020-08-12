#include <ros/ros.h>

// This include Vector3 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void movingFrameCb(const ros::TimerEvent&)
{

  static uint32_t counter = 0;
  static tf2_ros::TransformBroadcaster br;
  
  // [IMAN] I think Transform class is for math purposes, using it with 
  // eigen and other non-stamped class
  // for example use in the conversion 
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "map";
  t.child_frame_id = "moving_frame";
  
  tf2::convert(tf2::Vector3(5.0*cos(float(counter)/140.0), 5.0*sin(float(counter)/140.), sin(float(counter)/140.0)*2.0), t.transform.translation);
  tf2::convert(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), t.transform.rotation);
  
//  t.transform.translation.x = 0;
//  t.transform.translation.y = 0;
//  t.transform.translation.z = sin(float(counter)/140.0)*2.0;
//  t.transform.rotation.x = 0.0;
//  t.transform.rotation.y = 0.0;
//  t.transform.rotation.z = 0.0;
//  t.transform.rotation.w = 1.0;

  //This is where I can change the affine of the frame 
  //since the callback is periodically using the timer
  //t.transform.translation =  tf2::Vector3(0.0, 0.0, sin(float(counter)/140.0)*2.0);
  //t.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  
  //geometry_msgs::TransformStamped t_msgs;
  //tf2::convert(t, t_msgs);  
  
  // there is not tf2::StampedTransform use geometry_msgs::TransformStamped
  br.sendTransform(t); 
  counter++;
}

void movingFrameCb2(const ros::TimerEvent&)
{

  static uint32_t counter = 0;
  static tf2_ros::TransformBroadcaster br;
  // [IMAN] I think Transform class is for math purposes, using it with 
  // eigen and other non-stamped class
  // for example use in the conversion 
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "map";
  t.child_frame_id = "moving_frame2";
  
  tf2::convert(tf2::Vector3(5.0*cos(float(counter)/140.0), 5.0*sin(float(counter)/140.), 0.0), t.transform.translation);
  tf2::convert(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), t.transform.rotation);
  
//  t.transform.translation.x = 0;
//  t.transform.translation.y = 0;
//  t.transform.translation.z = sin(float(counter)/140.0)*2.0;
//  t.transform.rotation.x = 0.0;
//  t.transform.rotation.y = 0.0;
//  t.transform.rotation.z = 0.0;
//  t.transform.rotation.w = 1.0;

  //This is where I can change the affine of the frame 
  //since the callback is periodically using the timer
  //t.transform.translation =  tf2::Vector3(0.0, 0.0, sin(float(counter)/140.0)*2.0);
  //t.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  
  //geometry_msgs::TransformStamped t_msgs;
  //tf2::convert(t, t_msgs);  
  
  // there is not tf2::StampedTransform use geometry_msgs::TransformStamped
  br.sendTransform(t); 
  counter++;
}

void publishPoint(const ros::TimerEvent&)
{
  static ros::NodeHandle nh;
  static ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("moving_frame2_point",5);
  geometry_msgs::PointStamped point;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "moving_frame2";
  point.point.x = 1.0;
  point.point.y = 1.0;
  point.point.z = 1.0;
  pub.publish(point);
}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "tf_moving_frames");
  ros::NodeHandle n;
  
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), movingFrameCb);
  ros::Timer frame_timer2 = n.createTimer(ros::Duration(0.034), movingFrameCb2);
  ros::Timer point_timer = n.createTimer(ros::Duration(0.001), publishPoint);
  ros::spin();
}
