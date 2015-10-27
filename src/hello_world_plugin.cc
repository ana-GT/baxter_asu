/**
 * @file helloworld plugin
 * @date Friday, October 23rd, 2015
 */
#include <gazebo/gazebo.hh>

namespace gazebo {

  class WorldPlugin_HelloWorld : public WorldPlugin {
    public: WorldPlugin_HelloWorld() : WorldPlugin() {
      printf("Hello Houston from the Mother Ship! \n");
    }

    public: void Load( physics::WorldPtr _world, sdf::ElementPtr _sdf ) {
      
    }

  };

  GZ_REGISTER_WORLD_PLUGIN( WorldPlugin_HelloWorld )

}
