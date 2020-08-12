#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
  protected:
    ros::NodeHandle nh_;
    
    //Nodehandle instance must be created before this line. Otherwise error occurs.
    actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;
    std::string action_name_;

    //Create message that are used to pubslished feedback/result
    actionlib_tutorials::FibonacciFeedback feedback_;
    actionlib_tutorials::FibonacciResult result_;
    int i_last_;
  public:
    FibonacciAction(std::string name) :
      as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), 
      action_name_(name),
      i_last_(1)
    {
      as_.registerPreemptCallback(boost::bind(&FibonacciAction::switchCb,this));
      as_.start();
    }

    ~FibonacciAction(void)
    {
    }
    
    void switchCb()
    {
      ROS_WARN_STREAM("Switching");
    }
    void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
    {
      // helper variables
      ros::Rate r(1);
      bool success = true;
      feedback_.sequence.clear();
      feedback_.sequence.push_back(0);
      feedback_.sequence.push_back(1);

      //publish info to the console for the user
      ROS_INFO("%s: Executing, creating Fibonacci sequence of order %i with seeds %i, %i",
                    action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

      //Start executing the action 
      
      for(int i=i_last_; i<=goal->order; i++)
      {
        
        if (as_.isPreemptRequested() || !ros::ok())
        {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          //Set the action state to preempted
          as_.setPreempted();
          success = false;
          i = i_last_;
        }
        
        feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
        // publish the feedback
        as_.publishFeedback(feedback_);
        //Sleep so the sequence is computed 1 Hz for demonstration
        r.sleep();
      }

      if(success)
      {
        result_.sequence = feedback_.sequence;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // Set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }
};

int main (int argc, char** argv)
{
  //fibonacci is the name of the node
  ros::init(argc, argv, "fibonacci");

  //fibonacci is the namespace of the action messages
  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}


