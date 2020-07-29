#include "gazebo_moving_drone.h"

using namespace std;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MoveDrone)

bool poseEquals(ignition::math::Pose3d posA, ignition::math::Pose3d posB, float tol);

/////////////////////////////////////////////////

gazebo::MoveDrone moveDrone;

MoveDrone::MoveDrone()
{
}

MoveDrone::~MoveDrone()
{
}

void MoveDrone::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&MoveDrone::OnUpdate, this, _1));

        // Exit if no ROS
        if (!ros::isInitialized())
        {
            gzerr << "Not loading Drone_ball movement plugin since ROS hasn't been "
                  << "properly initialized.  Try starting gazebo with ros plugin:\n"
                  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzwarn << "[gazebo_moving_platform] Please specify a robotNamespace.\n";

        this->node_handle_ = new ros::NodeHandle(namespace_);

        this->moving_sub = this->node_handle_->subscribe("clock", 10, &MoveDrone::getSimulationClockTime, this);

        //this->reset_sub_ = this->node_handle_->subscribe("/moving_platform/reset", 10, &ModelPush::resetCallback, this);

        // Get initial pose
        initial_pose_ = this->model->WorldPose();

        //Move to desired origin
        ignition::math::Pose3d pose = this->model->WorldPose();
        pose.Pos() = origin;
        this->model->SetWorldPose(pose);
    }

    // Called by the world update start event
void MoveDrone::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Sim time
        double sec = this->time_sec + this->time_nsec / 1E9;

        //Cancel rotation
        math::Pose pose = this->model->WorldPose();
        pose.Rot().X() = 0;
        pose.Rot().Y() = 0;
        pose.Rot().Z() = 0;
        pose.Rot().W() = 1;
        
        //print info
        #ifdef DEBUG
            std::cout << endl << "sec: " << sec << "; POS - X: " << pose.Pos().X() << "; Y: " << pose.Pos().Y() << "; Z: " << pose.Pos().Z() << std::endl;
        #endif

        this->model->SetWorldPose(pose);

        //Trajectory generation
        //Comment corresponding defines in .h to enable/disable movement in each axis.
        //X axis parameters are required for the plugin to function correctly

        float kf = 1.0; //Frecuency compensation to keep everything in sync
        vel_z_ = GRAVITY*STEP; //Gravity compensation

        //X avis movement
        #if defined(AMPL_X) && defined(FREQ_X)
            A_ = AMPL_X;
            T_ = PERIOD/FREQ_X; //fix X as base
            kf = 1 / (T_ * FREQ_X);
            w_ = 2 * M_PI / T_;
            vel_x_ =  A_*w_*cos(w_*sec);
        #endif

        //Y axis movement
        #if defined(AMPL_Y) && defined(FREQ_Y)
            A_ = AMPL_Y;

            #if !defined(AMPL_X) || !defined(FREQ_X)
                kf = 1 / (T_ * FREQ_Y);
            #endif

            T_ = 1 / (kf*FREQ_Y);
            w_ = 2 * M_PI / T_;
            vel_y_ =  A_*w_*cos(w_*sec);
        #endif

        //Z axis movement
        #if defined(AMPL_Z) && defined(FREQ_Z)
            A_ = AMPL_Z;
            T_ = 1 / (kf*FREQ_Z);
            w_ = 2 * M_PI / T_;
            vel_z_ += A_*w_*cos(w_*sec);
        #endif

        //Update model        
        this->model->GetLink("base_link")->SetLinearVel({vel_x_, vel_y_, vel_z_}); //Update velocities
        this->model->GetLink("base_link")->SetAngularVel({0, 0, 0}); //Keep model from rotating

        //Info
        #ifdef DEBUG
            cout << "VEL - X: " << vel_x_ << "; Y: " << vel_y_ << "; Z: " << vel_z_ << endl;
        #endif
    }


void MoveDrone::getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg){

    //Get sim time
    ros::Time time = msg->clock;

    //updatre previous time
    this->time_sec_prev = this->time_sec;
    this->time_nsec_prev = this->time_nsec;

    //update current time
    this->time_sec = time.sec - initial_time_sec;
    this->time_nsec = time.nsec - initial_time_nsec;

    //Get model pose
    math::Pose pose = this->model->WorldPose();

    // Restart the counter
    if (this->time_sec >= PERIOD*0.8 && poseEquals(initial_pose_, pose, 0.1)){
        
        //Set current time as starting time
        initial_time_sec = time.sec;
        initial_time_nsec = time.nsec;

        //Reset position
        pose.Pos() = origin;
        this->model->SetWorldPose(pose);

        cout << " ******** Position and time reset ********" << endl;
    }
}

bool poseEquals(ignition::math::Pose3d posA, ignition::math::Pose3d posB, float tol)
{
    math::Pose diff = posA - posB;
    if(abs(diff.pos.x) > tol || abs(diff.pos.y) > tol || abs(diff.pos.z) > tol)
        return false;
    else
        return true;
}
