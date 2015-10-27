#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {

  class PushModel : public ModelPlugin {
    
    
    public: void Load( physics::ModelPtr _parent, sdf::ElementPtr ) {
  
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&PushModel::OnUpdate, this, _1));
    }

    //-- Called by the world update start event
    public: void OnUpdate( const common::UpdateInfo &_info ) {
      this->model->SetLinearVel( math::Vector3(0.3, 0, 0) );
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

  }; 

  // REGISTER this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PushModel)

}
