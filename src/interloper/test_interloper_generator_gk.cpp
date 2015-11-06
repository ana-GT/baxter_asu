
#include <Eigen/Core>
#include <list>
#include <motion_control/msgs/bimanual_msgs.h>

#include <ach.h>

int main( int argc, char* argv[] ) {

  // Open chan
  struct sns_msg_bimanual* msg = NULL;
  int N = 7;
  std::list<Eigen::VectorXd> path;

  // Relleno de 3 points
  Eigen::VectorXd p(N);
  p << 0, 0, 0, 0, 0, 0, 0;
  path.push_back(p);
  p << -0.11, -0.62, -1.15, 1.32, 0.80, 1.27, 2.39;
  path.push_back(p);
  p = p*0.75;
  path.push_back(p);
  p = p*1.67;
  //path.push_back(p);

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
    msg->mode = 1 - msg->mode;
    r = ach_put( &chan, msg, sns_msg_bimanual_size(msg) );
    if( r != ACH_OK ) { printf("Error sending message \n"); }
    // Sleep
    usleep(0.5*1e6);
  }

  return 0;
}
