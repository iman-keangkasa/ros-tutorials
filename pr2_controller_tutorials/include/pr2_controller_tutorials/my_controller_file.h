//Capturing data tutorials
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_controller_tutorials/MyStateMessage.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_controller_tutorials/SetAmplitude.h>

// Adding PID controllers tutorials
#include <control_toolbox/pid.h>


namespace my_controller_ns{
  // Capturing data tutorials
  // We declare the length of storage buffer
  // 5000 samples corresponse to a 5 second time window
  enum
  {
    StoreLen = 5000
  };

  class MyControllerClass: public pr2_controller_interface::Controller
  {
    private:
      pr2_mechanism_model::JointState* joint_state_;
      double init_pos_;

      // Service to set amplitude */
      bool setAmplitude(pr2_controller_tutorials::SetAmplitude::Request & req,
                        pr2_controller_tutorials::SetAmplitude::Response & resp);
      
      // Adding PID controllers tutorials
      // time_of_last_cycle_ will updates the last update
      // process so that delta t can be ascertained. Delta t 
      // is use to get i_gain and d_gain
      //
      // We will use robot_ to access time and to set time
      control_toolbox::Pid pid_controller_;
      pr2_mechanism_model::RobotState *robot_;
      ros::Time time_of_last_cycle_;

      double amplitude_;
      ros::ServiceServer srv_;

      // Capturing data tutorials
      bool capture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
      ros::ServiceServer capture_srv_;
      ros::Publisher mystate_pub_;
      pr2_controller_tutorials::MyStateMessage storage_[StoreLen];
      volatile int storage_index_;

    public:
      virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
      virtual void starting();
      virtual void update();
      virtual void stopping();
      void printPID();
  };

}
