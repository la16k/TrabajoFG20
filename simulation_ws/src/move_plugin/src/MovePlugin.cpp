#include "MovePlugin.h"

using namespace std;
using namespace gazebo;

//Registro del plugin en Gazebo
GZ_REGISTER_MODEL_PLUGIN(Move) 

void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);

gazebo::Move move_;

/// Constructor
/// \brief Move::Move
///
Move::Move()
{
}

/// Destructor
/// \brief RayPlugin::~RayPlugin
///
Move::~Move()
{
}

void Move::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&Move::OnUpdate, this, _1));

        this->node_handle_ = new ros::NodeHandle(namespace_);

        this->moving_sub = this->node_handle_->subscribe("clock", 10, &Move::getSimulationClockTime, this);

        // this->reset_sub_ = this->node_handle_->subscribe("/moving_platform/reset", 10, &Move::resetCallback, this);

        counter = 0;
            
        initial_pose_ = this->model->GetWorldPose();
        update_ = false;
    }

void Move::OnUpdate(const common::UpdateInfo &)
    {
        common::Time time = common::Time::GetWallTime();
        double sec = (move_.time_sec - move_.time_sec_prev) + (move_.time_nsec - move_.time_nsec_prev) / MILLION;

         A_ = 2.7;
         T_ = 5 * M_PI;
         w_ = 2 * M_PI / T_;

         vel_ = A_ * w_ * cos(w_ * sec);
        // vel_ = 0.2;
        this->model->SetLinearVel(math::Vector3(0, vel_, 0));
    }

void Move::getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg)
    {
        ros::Time time = msg->clock;
        math::Pose pose = this->model->GetWorldPose();

        double sec = (move_.time_sec - move_.time_sec_prev) + (move_.time_nsec - move_.time_nsec_prev) / MILLION;
    
        if(!update_)   {
            ros::Time time = msg->clock;
            ini_time_sec = time.sec;
            ini_time_nsec = time.nsec;

            update_ = true;
        }

        move_.time_sec = time.sec - ini_time_sec;
        move_.time_nsec = time.nsec - ini_time_nsec;
    }