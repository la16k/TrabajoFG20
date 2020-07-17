#include "MovePlugin.h"

using namespace std;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Move)

void getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg);

gazebo::Move _move;

/////////////////////////////////////////////////
/// \brief Move::Move
///
Move::Move()
{
}

/////////////////////////////////////////////////
/// \brief RayPlugin::~RayPlugin
///
Move::~Move()
{
}

void Move::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&Move::OnUpdate, this, _1));

        // Exit if no ROS
        if (!ros::isInitialized())
        {
            gzerr << "Not loading Optical Flow plugin since ROS hasn't been "
                  << "properly initialized.  Try starting gazebo with ros plugin:\n"
                  << "  gazebo -s libgazebo_ros_api_plugin.so\n";
            return;
        }

        if (_sdf->HasElement("robotNamespace"))
          namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
          gzwarn << "[gazebo_moving_platform] Please specify a robotNamespace.\n";

        this->node_handle_ = new ros::NodeHandle(namespace_);

        this->moving_sub = this->node_handle_->subscribe("clock", 10, &Move::getSimulationClockTime, this);

        // this->reset_sub_ = this->node_handle_->subscribe("/moving_platform/reset", 10, &Move::resetCallback, this);

        counter = 0;

        // Get initial pose
        initial_pose_ = this->model->WorldPose();
        
        // Init variables
        updated_ = false;
    }

    // Called by the world update start event
void Move::OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // Get Sim time
        common::Time _time = common::Time::GetWallTime();
        double sec = (_move.time_sec - _move.time_sec_prev) + (_move.time_nsec - _move.time_nsec_prev) / MILLION;
        //cout << sec << endl;
// #ifdef SLOW_MP
	// Slow
        A_ = 2.7;
        T_ = 13.5 * M_PI;
        w_ = 2 * M_PI / T_;
// #endif

// #ifdef MID_MP
// 	// Mid
//         A_ = 2.7;
//         T_ = 4.5 * M_PI;
//         w_ = 2 * M_PI / T_;
// #endif

// #ifdef FAST_MP
// 	// Fast
//         A_ = 2.7;
//         T_ = 3.75 * M_PI;
//         w_ = 2 * M_PI / T_;
// #endif

// #if defined SLOW_MP || defined MID_MP || defined FAST_MP
        // Compute velocity
        vel_ = A_*w_*cos(w_*sec);

        // vel_ = 2;

        // Apply a small linear velocity to the model.
        this->model->SetLinearVel(ignition::math::Vector3d(0, vel_, 0));
// #endif

// #ifdef EIGHT_SHAPE
//         // 8-shape trajectory
//         double P_a=3.0;
//         double P_b=1.5;
//         double P_c=0.5;
//         double P_n=-1.2;
	   
//         P_T_=15;
	   
//         double P_w1=2*M_PI/P_T_;
//         double P_w2=P_w1/2;
//         double P_w3=0;

//         double t=sec;
//         double a=P_a;
//         b_=P_b;
//         double c=P_c;

//         w1_=P_w1;
//         double w2=P_w2;
//         double w3=P_w3;
//         double n=P_n;
//         //out=[psi_r dot_psi_r ddot_psi_r];
//         //ytraj=[a*cos(w2*t);b*sin(w1*t);n+c*sin(w3*t);0];
//         vel_x_ = b_*w1_*cos(w1_*(t- P_T_ / 2.0) );
//         double vel_y = -a*w2*sin(w2*(t- P_T_ / 2.0));
//         double vel_z = c*w3*cos(w3*t);
//         T_ = 2 * P_T_;

//         // Apply a small linear velocity to the model.
//         this->model->SetLinearVel(math::Vector3(vel_x_, vel_y, vel_z));
//         this->model->SetAngularVel(math::Vector3(0.0, 0.0, 0.1));

//         //ydottraj=[-a*w2*sin(w2*t);b*w1*cos(w1*t);c*w3*cos(w3*t);0];
//         //yddottraj=[-a*w2*w2*cos(w2*t);-b*w1*w1*cos(w1*t);-c*w3*w3*sin(w3*t);0];
//  #endif

// #ifdef RUNAWAY
//         // Runaway trajectory
//         // Apply a small linear velocity to the model.
//         if (sec > SETTLELING_TIME){
//             vel_ = - RUNAWAY_SLOPE * sec;
//             if (fabs(vel_) > MAX_VEL)
//                 vel_ = - MAX_VEL;

//             this->model->SetLinearVel(math::Vector3(0, vel_, 0));
//         }
// #endif
        
//        if (counter < COUNTER_MAX){
//          counter++;
//          modelPush.time_sec_prev = modelPush.time_sec;
//          modelPush.time_nsec_prev = modelPush.time_nsec;
//        }
//        else
//        {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(math::Vector3(0, vel_, 0));
	    //this->model->SetAngularVel(math::Vector3(0, 0, 0.2));
//        }
        // Get world pose
        //       math::Pose pose = this->model->GetWorldPose();
        //       math::Vector3 pos = pose.pos;
        //       cout << "Moving Platform current pose" << endl;
        //       cout << "x: " << pos.x << "y: " << pos.y << "z: " << pos.z << endl;



    }


void Move::getSimulationClockTime(const rosgraph_msgs::Clock::ConstPtr& msg){

    ros::Time _time = msg->clock;
    ignition::math::Pose3d pose = this->model->WorldPose();

    double sec = (_move.time_sec - _move.time_sec_prev) + (_move.time_nsec - _move.time_nsec_prev) / MILLION;

// #ifndef RUNAWAY
    // Restart in the origin
    if (/*(pose.pos.y > -0.01) && (pose.pos.y < 0) && */ (sec >= (T_)) && !updated_){
        initial_time_sec = _time.sec;
        initial_time_nsec = _time.nsec;

        // Reorient model
        // pose.Pose3.x = initial_pose_.Pose3.x;
        // pose.Pos.y = initial_pose_.Pos.y;
        // pose.Pos.z = initial_pose_.Pos.z;
//        pose.rot.x = 0;
//        pose.rot.y = 0;
//        pose.rot.z = 0;
//        pose.rot.w = 1;
        this->model->SetWorldPose(pose);

                updated_ = true;
    }
    
    if (fabs(vel_) < 0.1)	updated_ = false;
// #else
//     if(!updated_){
//         // Restart initial time
//         ros::Time time = msg->clock;
//         initial_time_sec = time.sec;
//         initial_time_nsec = time.nsec;

//         // Reinit
//         updated_ = true;
//     }
// #endif

    _move.time_sec = _time.sec - initial_time_sec;
    _move.time_nsec = _time.nsec - initial_time_nsec;

    //cout << "time secs: " << modelPush.time_sec << endl;
}

// #ifdef RUNAWAY
// void ModelPush::resetCallback(const std_msgs::Bool::ConstPtr& msg){
//     // Restore pose of the model to the origin
//     math::Pose pose = this->model->GetWorldPose();

//     pose.pos.x = initial_pose_.pos.x;
//     pose.pos.y = initial_pose_.pos.y;
//     pose.pos.z = initial_pose_.pos.z;
//     pose.rot.x = 0;
//     pose.rot.y = 0;
//     pose.rot.z = 0;
//     pose.rot.w = 1;
//     this->model->SetWorldPose(pose);

//     // Restart time
//     updated_ = false;


// }
// #endif
