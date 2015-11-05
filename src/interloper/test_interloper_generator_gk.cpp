
#include <Eigen/Core>
#include <list>
#include <motion_control/msgs/bimanual_msgs.h>

#include <ach.h>

int main( int argc, char* argv ) {

  // Open chan
  struct sns_msg_bimanual* msg = NULL;
  int N = 7;
  int len = 3;
  std::list<Eigen::VectorXd> path;

  for( int i = 0; i < len; ++i ) {
    Eigen::VectorXd p(N);
    for( int j = 0; j < N; ++j ) {
      p(j) = 0.125*j;
    }
    path.push_back(p);
  }
  msg = sns_msg_bimanual_alloc( path.size(), 0, 
				N );
  int counter = 0;
  for( std::list<Eigen::VectorXd>::iterator it = path.begin(); it != path.end(); ++it ) {
    for( int j = 0; j < (*it).size(); ++j ) {
      msg->x[counter] = (*it)(j);
      counter++;
    }
  }

  // Open channel
  ach_channel_t chan;
  enum ach_status r = ach_open( &chan, "bimanual_chan", NULL);
  
  if( r != ACH_OK ) {
    printf("Error opening bimanual_chan \n");
    return 1; 
  }

  while(true) {
    
    // Send message
    r = ach_put( &chan, msg, sns_msg_bimanual_size(msg) );
    if( r != ACH_OK ) { printf("Error sending message \n"); }
    // Sleep
    usleep(0.1*1e6);
  }

  return 0;
}
