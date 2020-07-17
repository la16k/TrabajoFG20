#include <gazebo/gazebo.hh> 
#include <gazebo/physics/physics.hh> 
#include <gazebo/common/common.hh> 
#include <stdio.h>
#include <iostream>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <boost/bind.hpp>
#include <stdlib.h>

// #include <ignition/math/Quaternion.hh>
// #include <ignition/math/Vector3.hh>
// #include <ignition/math/config.hh>

#define MILLION 1E9
// #define COUNTER_MAX 20000

// Choose the platform mode
//#define SLOW_MP
//#define MID_MP
//#define FAST_MP
//#define EIGHT_SHAPE
// #define RUNAWAY

// #ifdef RUNAWAY
// #include <std_msgs/Bool.h>
// #endif

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

    // Called by the world update start event
public:
    void OnUpdate(const common::UpdateInfo & /*_info*/);
    void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);
// #ifdef RUNAWAY
//  void resetCallback(const std_msgs::Bool::ConstPtr& msg); 
// #endif

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
    
private:
    ros::NodeHandle *node_handle_;
    ros::Subscriber moving_sub;
    std::string namespace_;

// #ifdef RUNAWAY
//     ros::Subscriber reset_sub_;
// #endif
public:
    double time_sec;
    double time_nsec;
    double time_sec_prev;
    double time_nsec_prev;
    double initial_time_sec;
    double initial_time_nsec;
    double vel_, vel_x_, b_, w1_, P_T_;
    bool updated_;
    int counter;
    ignition::math::Pose3d initial_pose_;
    double A_;
    double T_;
    double w_;

// #ifdef RUNAWAY
//     const double SETTLELING_TIME = 6.0;
//     const double RUNAWAY_SLOPE = 0.3;
//     const double MAX_VEL = 1.5; // m/s
// #endif

};
}