#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PoseDrawer
{
  private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    ros::NodeHandle n_;
    message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
    tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

  public:
    PoseDrawer() :
      tf2_(buffer_), 
      target_frame_("moving_frame"),
      tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
    {
      point_sub_.subscribe(n_, "moving_frame2_point", 10);
      tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgsCallback, this, _1) );
    }

    // Callback to register with tf2_ros::MessageFilter to be called when transform are 
    // available

    void msgsCallback(const geometry_msgs::PointStampedConstPtr& point_ptr)
    {
      geometry_msgs::PointStamped point_out;
      try
      {
        buffer_.transform(*point_ptr, point_out, target_frame_);
        ROS_INFO("Point of the moving_frame2 in moving_frame position(x:%4.3f y:%4.3f z:%4.3f)\n",
                  point_out.point.x,
                  point_out.point.y,
                  point_out.point.z);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("Failure %s\n", ex.what());
      }
    }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer");
  PoseDrawer pd;
  ros::spin();
  return 0;
}


