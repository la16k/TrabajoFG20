#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {
    class PlanarMover : public ModelPlugin {
        public: void Load(physics::ModelPtr _parent, sdf::ElemetPtr) {
            this->model = _parent;
            this->updateConnection = event::Events:ConnectWorldUpdateBegin(
                std::bind(&PlannarMover::OnUpdate, this)
            );
            this->old_secs =ros::Time::now().toSec();

            std::string plannar_pos_topicName = "/cmd_vel";
            
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "plannar_rosnode",
                    ros::init_options::NoSiginHandler);
            }

            this->rosNode.reset(new ros::NodeHandle("plananr_rosnode"));

            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(
                    plannar_pos_topicName,
                    1,
                    boost::bind(&PlannarMover::OnRosMsg_Pos, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->suscribe(so);
                
            this->rosQueueThread =
                std::thread(std::bind(&PlannarMover::QueueThread, this));

            ROS_WARN("Loaded PlannarMover Plugin with parent... %s, only X axis freq supported in this V-1.0", this->model->GetName().c_str());
        }

        public: void OnUpdate(){
            
        }

        void MoveModelsPlane(float linear_x_vel, float linear_y_vel, float linear_z_vel, float angular_x_vel, float angular_y_vel, float angular_z_vel) {
            std::string model:name = this->model->GetName();

            ROS_DEBUG("Moving model = %s", model_name.c_str());

            this->model->SetLinearVel(ignition::math::Vector3d(linear_x_vel, linear_y_vel, linear_z_vel));
            this->model->SetAngularVel(ignition::math::Vector3d(angular_x_vel, angular_y_vel, angular_z_vel));

            ROS_DEBUG("Moving model = %s... END", model_name.c_str());
        }

        public: void OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &_msg){
            this->MoveModelsPlante(_msg->linear.x, _msg->linear.y, _msg->linear.z, _msg->angular.x, _msg->angular.y, _msg->angular.z);
        }

        private: void QueueThread(){
            static const double timeout = 0.01;
            while(this->rosNode->ok()){
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        } 

        private: physics::ModelPtr model;

        private: event::ConnectionPtr updateConnection;

        double old_secs;

        int direction = 1;
        double x_axis_pos = 1.0;
        double y_axis_pos = 1.0;

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::Subscriber rosSub; 

        private: ros::CallbackQUEUE rosQueue;

        private: std::thread rosQueueThread;

        private: ros::Subscriber rosSub2;
        
        private: ros::CallbackQueue rosQueue2;

        private: ros::thread rosQueueThread2;
    };

    GZ_REGISTER_MODEL_PLUGIN(PlannarMover)
}