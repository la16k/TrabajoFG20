#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

//////////////////////////////////////////////////////////////////
/*
TO DO LIST: 
- Enable for random trajectories
# define RAMDOM 0

*/
//////////////////////////////////////////////////////////////////

//Simulation parameters (Don't change!)

#define GRAVITY 10.0286 //Gravity used in Gazebo sim. Do not change, or the drone will slowly float away or fall to the ground
#define STEP 0.01 //Time step used in gazebo sim (100 Hz)

//////////////////////////////////////////////////////////////////

// Trajectory parameters (change as you wish!)

#define PERIOD 80 //Period for base movement (X) in secons

//Drone origin in meters
#define ORI_X -2
#define ORI_Y 0
#define ORI_Z 0

//Movement amplitude in meters
#define AMPL_X 50
#define AMPL_Y 3
//#define AMPL_Z 0.5

//Movement frequency factor. The 3D trajectory is defined by the ratios of these numbers (Lissajous figures)
#define FREQ_X 3
#define FREQ_Y 2
//#define FREQ_Z 2

//////////////////////////////////////////////////////////////////

//Toggle debug info
#define DEBUG

using namespace std;

namespace gazebo
{

class MoveDrone : public ModelPlugin
{

/// \brief Constructor
    public: MoveDrone();

    /// \brief Destructor
    public: virtual ~MoveDrone();

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
public:
    void OnUpdate(const common::UpdateInfo & /*_info*/);
    void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);

    //void resetCallback(const std_msgs::Bool::ConstPtr& msg);


    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
    
private:
    ros::NodeHandle *node_handle_;
    ros::Subscriber moving_sub;
    std::string namespace_;

#ifdef RUNAWAY
    ros::Subscriber reset_sub_;
#endif
public:

    //Time variables (seconds and nanoseconds)
    double time_sec;
    double time_nsec;
    double time_sec_prev;
    double time_nsec_prev;
    double initial_time_sec;
    double initial_time_nsec;

    //Drone initial position (defined by .world file)
    ignition::math::Pose3d initial_pose_;

    //Desired trajectory origin position
    ignition::math::Vector3d origin{ORI_X, ORI_Y, ORI_Z};

    //Drone initial velocities (using gravity compensation)
    double vel_x_ {0.0}, vel_y_ {0.0}, vel_z_ {GRAVITY*STEP};

    //Sinusoidal trajectory variables
    double A_;
    double T_;
    double w_;

};

}
