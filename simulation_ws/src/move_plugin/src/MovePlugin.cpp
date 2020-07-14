#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
    class Move : public ModelPlugin 
    {
        public: void Load(physics::ModelPtr columna, sdf::ElementPtr _sdf)
            {
                this->model = columna;

                gazebo::common::PoseAnimationPtr animacion(
                   new gazebo::common::PoseAnimation("test", 10.0, true)
                );

                gazebo::common::PoseKeyFrame *key;

                key = animacion->CreateKeyFrame(0);
                key->Translation(ignition::math::Vector3d(-0.885, 0.5, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));

                key = animacion->CreateKeyFrame(2.0);
                key->Translation(ignition::math::Vector3d(-0.885, 0, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));
                
                key = animacion->CreateKeyFrame(4.0);
                key->Translation(ignition::math::Vector3d(-0.885, -0.5, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));

                key = animacion->CreateKeyFrame(6.0);
                key->Translation(ignition::math::Vector3d(-0.885, 0, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));

                key = animacion->CreateKeyFrame(8.0);
                key->Translation(ignition::math::Vector3d(-0.885, 0.5, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));

                key = animacion->CreateKeyFrame(10.0);
                key->Translation(ignition::math::Vector3d(-0.885, 0, 0));
                key->Rotation(ignition::math::Quaterniond(0, 0, 0));

                columna->SetAnimation(animacion);
            }

            private: physics::ModelPtr model;

            private: event::ConnectionPtr updateConection;
    };

    GZ_REGISTER_MODEL_PLUGIN(Move)
}
