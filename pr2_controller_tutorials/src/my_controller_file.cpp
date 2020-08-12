#include "pr2_controller_tutorials/my_controller_file.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyControllerClass, pr2_controller_interface::Controller)

namespace my_controller_ns {
  
  // Service to set amplitude


  // controller initialization in non-realtime
  bool MyControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    std::string joint_name;
    
    if (!n.getParam("joint_name", joint_name))
    {
      ROS_ERROR("No joint given in namesapce: '%s'", n.getNamespace().c_str());
      return false;
    }

    joint_state_ = robot->getJointState(joint_name);
    
    if (!joint_state_)
    {
      ROS_ERROR("MyController could not find joint named '%s'", joint_name.c_str());
      return false;
    }
    
    srv_ = n.advertiseService("set_amplitude", &MyControllerClass::setAmplitude, this);
    amplitude_ = 0.5;
    
    // copy robot pointer so we can access time
    robot_ = robot;

    //initialize PID controller
    // The init method takes a NodeHandle that specifies the namespace in which 
    // the pid parameters are specified. In this case the namespace is the controller name
    // plus the string "pid_parameters". This namespace is important to set the pid values 
    // in the launch file.
    if (!pid_controller_.init(ros::NodeHandle(n, "pid_parameters")))
    {
      ROS_ERROR("MyControllerClass could not construct PID controller for joint %s", joint_name.c_str());
      return false;
      //Display the PID gains 
    }

    // Capturing data from controller tutorials
    // to prevent interruption to the realtime controller (a realtime hardware interface control system)
    // we use buffer with the size of StoreLen to store the state and the performance of the controller
    capture_srv_ = n.advertiseService("capture", &MyControllerClass::capture, this);
    mystate_pub_ = n.advertise<pr2_controller_tutorials::MyStateMessage>("mystate_topic", StoreLen);
    storage_index_ = StoreLen;

    return true;
  }

  //controller startup in realtime
  void MyControllerClass::starting()
  {
    init_pos_ = joint_state_->position_;
    time_of_last_cycle_ = robot_->getTime();
    pid_controller_.reset();
    printPID();
  }

  // Controller update loop in realtime
  void MyControllerClass::update()
  {
    double desired_pos = init_pos_ + amplitude_ * sin(ros::Time::now().toSec());
    double current_pos = joint_state_->position_;
    
    // getTime() is a function from hardware_interface, it is different 
    // from ros::Time::now(). The time provided by the robot model is
    // the same for all controllers, and is the time at which
    // the controller manager started triggering all controllers
    ros::Duration dt = robot_->getTime() - time_of_last_cycle_;

    //joint_state_->commanded_effort_ = -10 * (current_pos - desired_pos);

    joint_state_->commanded_effort_ = pid_controller_.updatePid(current_pos-desired_pos, dt);
    printPID();
    
    int index = storage_index_;
    if( (index>=0) && (index < StoreLen))
    {
      storage_[index].dt                      = 0.0;
      storage_[index].position                = joint_state_->position_;
      storage_[index].desired_position        = desired_pos;
      storage_[index].velocity                = joint_state_->velocity_;
      storage_[index].desired_velocity        = 0.0;
      storage_[index].commanded_effort        = joint_state_->commanded_effort_;
      storage_[index].measured_effort         = joint_state_->measured_effort_;

      storage_index_ = index+1;
    }
  }

  void MyControllerClass::stopping()
  {}
  
  void MyControllerClass::printPID()  
  {
    double p_gain, i_gain, d_gain, i_max, i_min;
    pid_controller_.getGains(p_gain, i_gain, d_gain, i_max, i_min);
    ROS_WARN("P:%f, I:%f, D:%f", p_gain, i_gain, d_gain);
    ROS_WARN("Integration min: %f",i_min);
    ROS_WARN("Integration min: %f",i_max);
  }

  bool MyControllerClass::setAmplitude(pr2_controller_tutorials::SetAmplitude::Request& req, 
                                       pr2_controller_tutorials::SetAmplitude::Response& resp)
  {
    if (fabs(req.amplitude) < 2.0)
    {
      amplitude_ =req.amplitude;
      ROS_INFO("Mycontroller: set amplitude to %f", req.amplitude);
    }
    else
      ROS_WARN("MyController: requested amplitude %f too large", req.amplitude);

    resp.amplitude = amplitude_;
    return true;
  }

  bool MyControllerClass::capture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
  {
    //record starting time
    ros::Time started = ros::Time::now();
    storage_index_ = 0;
    while (storage_index_ < StoreLen)
    {
      //Sleep for 1 ms so not to hog the CPU
      ros::Duration(0.001).sleep();

      //Make sure we dont hag here forever
      if( ros::Time::now() - started > ros::Duration(20.0))
      {
        ROS_ERROR("Waiting for buffer to fill up took longer than 20 seconds");
        return false;
      }
    }

    int index;
    for (index = 0; index < StoreLen; index++)
      mystate_pub_.publish(storage_[index]);
    return true;
  }



} //namespace

//We will add this class a plugin so that it can be loaded
//on the fly

/// Register controller to pluginlib

/*[DEPRICATED?]//
PLUGINLIB_DECLARE_CLASS(pr2_controller_tutorials,
                        MyControllerPlugin, 
                        my_controller_ns::MyControllerClass,
                        pr2_controller_interface::Controller)
*/
