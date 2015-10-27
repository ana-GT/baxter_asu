#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

std::string model_string = "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>";

namespace gazebo {

  class Factory : public WorldPlugin {

    public: void Load( physics::WorldPtr _parent, sdf::ElementPtr _sdf ) {

      // 1. Insert model from file via function call
      _parent->InsertModelFile("model://box");

     // 2. Insert model from string via function call
     sdf::SDF sphereSDF;
     sphereSDF.SetFromString( model_string );
     
     sdf::ElementPtr model = sphereSDF.root->GetElement("model");
     model->GetAttribute("name")->SetFromString("unique_sphere");
     _parent->InsertModelSDF(sphereSDF);

     // 3. Insert model from file via message passing
     transport::NodePtr node( new transport::Node() );
     node->Init( _parent->GetName() );
     
     transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");
     msgs::Factory msg;
     msg.set_sdf_filename("model://cylinder");

     // Pose to initialize the model to
     msgs::Set( msg.mutable_pose(), math::Pose(math::Vector3(1,-2,0), math::Quaternion(0,0,0) ) );

     factoryPub->Publish( msg );

    }

  };

  GZ_REGISTER_WORLD_PLUGIN( Factory )

}
