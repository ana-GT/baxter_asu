#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <iostream>

void cb( ConstWorldStatisticsPtr &_msg ) {
  // Dump contents
  std::cout << _msg->DebugString();
}

int main( int argc, char** argv ) {

  gazebo::setupClient(argc,argv);
  //Create our node for communication
  gazebo::transport::NodePtr node( new gazebo::transport::Node() );
  node->Init();

  // Listen to gazebo world stat topics
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb );
  
  // Busy wait loop, replace with your own code
  while( true ) {
    gazebo::common::Time::MSleep(10);
  }

  // Shut everything down
  gazebo::shutdown();

}
