
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo {

  class TutorialFactory : public WorldPlugin {

    public: void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf ) {
      transport::NodePtr node( new transport::Node() );
      node->Init( _parent->GetName() );
      transport::PublisherPtr physicsPub = node->Advertise<msgs::Physics>("~/physics");

      msgs::Physics physicsMsg;
      physicsMsg.set_type(msgs::Physics::ODE);
      
      // Set the step time
      physicsMsg.set_max_step_size(0.01);
      // Change gravity
      msgs::Set( physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1) );
      physicsPub->Publish(physicsMsg);

    }

  };

  // Register
  GZ_REGISTER_WORLD_PLUGIN(TutorialFactory)

}
