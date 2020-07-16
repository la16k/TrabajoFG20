#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <boost/bind.hpp>

#define MILLION 1E9


using namespace std;

namespace gazebo
{
    class Move : public ModelPlugin 
    {
    /// \brief Constructor
    public: Move();

    /// \brief Destructor
    public: virtual ~Move();

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    public: 
        void OnUpdate(const common::UpdateInfo &);
        void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);
    
    private: physics::ModelPtr model;

    private: event::ConnectionPtr updateConnection;
    
    private:
    ros::NodeHandle *node_handle_;
    ros::Subscriber moving_sub;
    std::string namespace_;
    
    public:
        double time_sec, time_nsec;
        double time_sec_prev, time_nsec_prev;
        double ini_time_sec, ini_time_nsec;
        double vel_, vel_x_, b_, w1_, P_T_;
        int counter;
        bool update_;
        math::Pose initial_pose_;
        double A_;
        double T_;
        double w_;
    };
}


