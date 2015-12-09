/**
 * @file interloper.cpp
 */
#include <Python.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sns.h>
#include <stdio.h>
#include <ach.h>
#include <string.h>
#include <errno.h>
#include <list>
#include <ros/serialization.h>

#include <Eigen/Core>
#include <trajectory_msgs/JointTrajectory.h>

#include <motion_control/msgs/bimanual_msgs.h>
#include <motion_control/msgs/control_msgs.h>
#include <motion_control/trajectories/Trajectory.h>
#include <motion_control/trajectories/Path.h>

#define INTERLOPER_MAX_VEL_BAXTER 0.25
#define INTERLOPER_MAX_ACCEL_BAXTER 0.4
#define INTERLOPER_FREQ_HZ 100.0

// Helper
std::string getMsg( const std::list<Eigen::VectorXd> &_path, 
		    int _n_dofs, double _maxVel, 
		    double _maxAccel, double _freq );

/**
 * @function sendGatekeeperMsg
 * @brief Return the status (true or false) after executing trajectory
 */
static PyObject* sendGatekeeperMsg( PyObject* self, PyObject* args ) {
  char* limb; int res; char* chan_name;
  if( !PyArg_ParseTuple( args, "sis", &limb, &res, &chan_name ) ) {
    return NULL;
  }

  // Open channel
  ach_channel_t gatekeeper_msg_chan;
  enum ach_status r;
  
  // Open / start the communication
  r = ach_open( &gatekeeper_msg_chan, chan_name, NULL );  
  if( r != ACH_OK ) { 
    printf("ERROR: Opening %s - code: %s errno: %s \n", chan_name, ach_result_to_string(r), strerror(errno)  ); 
    return NULL; 
  }

  // Send gatekeeper msg
  gatekeeper_msg msg;
  sns_msg_header_fill( &msg.header );

  if( strcmp( limb,"left") == 0 ) { printf("Reply to left arm \n"); msg.type = ARM_LEFT_END; }
  else if( strcmp(limb,"right") == 0 ) { printf("Reply to right arm \n"); msg.type = ARM_RIGHT_END; }
  else if( strcmp(limb,"left_gripper") == 0 ) { printf("Reply to left gripper \n"); msg.type = HAND_LEFT_END; }
  else if( strcmp(limb,"right_gripper") == 0 ) { printf("Reply to right gripper \n"); msg.type = HAND_RIGHT_END; }

  msg.state = res;
  r = ach_put( &gatekeeper_msg_chan,
	       (void*)&msg, sizeof(msg) );

  ach_close( &gatekeeper_msg_chan );
  
  Py_RETURN_NONE;
}

/**
 * @brief readGatekeeperMsg
 */
static PyObject* readGatekeeperMsg( PyObject* self, PyObject* args ) {

  PyObject* gt_obj;
  Py_buffer gt_view;
  void* buf;

  if( !PyArg_ParseTuple( args, "O", &gt_obj ) ) {
    return NULL;
  }

  if( PyObject_GetBuffer( gt_obj, &gt_view, PyBUF_SIMPLE ) ) {
    printf("Error getting buffer \n");
    return NULL;
  } 
  
  buf = gt_view.buf;
  struct sns_msg_bimanual *msg = (struct sns_msg_bimanual*) buf;
  // Read
  int n_dofs, n_steps_left, n_steps_right, mode;
  n_dofs = msg->n_dof;

  // Left
  n_steps_left = msg->n_steps_left;
  n_steps_right = msg->n_steps_right;
  mode = msg->mode;
  printf("Mode from interloper: %d \n", mode);
  // Get trajectory
  int counter;
  std::list<Eigen::VectorXd> path[2];

  // LEFT TRAJECTORY
  if( mode == 0 ) {
    counter = 0;
    for( int i = 0; i < n_steps_left; ++i ) {
      Eigen::VectorXd p(n_dofs);
      for( int j = 0; j < n_dofs; ++j ) {
	p(j) = msg->x[counter];
	counter++;
      }
      path[0].push_back( p );
    } 
  } 
  // RIGHT TRAJECTORY
  else if( mode == 1 ) {
    counter = 0;
    for( int i = 0; i < n_steps_right; ++i ) {
      Eigen::VectorXd p(n_dofs);
      for( int j = 0; j < n_dofs; ++j ) {
	p(j) = msg->x[counter];
	counter++;
      }
      path[1].push_back( p );
    } 
  } 

  // Generate messages for both paths
  std::string str_msg[2];
  str_msg[0] = getMsg(path[0], n_dofs, INTERLOPER_MAX_VEL_BAXTER, INTERLOPER_MAX_ACCEL_BAXTER, INTERLOPER_FREQ_HZ );
  str_msg[1] = getMsg(path[1], n_dofs, INTERLOPER_MAX_VEL_BAXTER, INTERLOPER_MAX_ACCEL_BAXTER, INTERLOPER_FREQ_HZ );

  return Py_BuildValue("(s#is#)", str_msg[0].c_str(), str_msg[0].size(), mode, str_msg[1].c_str(), str_msg[1].size() );
}

/**
 * @brief readGatekeeperHandMsg
 */
static PyObject* readGatekeeperHandMsg( PyObject* self, PyObject* args ) {

  PyObject* gt_obj;
  Py_buffer gt_view;
  void* buf;
  double left_pos = 0; double right_pos = 0;

  if( !PyArg_ParseTuple( args, "O", &gt_obj ) ) {
    return NULL;
  }

  if( PyObject_GetBuffer( gt_obj, &gt_view, PyBUF_SIMPLE ) ) {
    printf("Error getting buffer \n");
    return NULL;
  } 
  
  buf = gt_view.buf;
  struct sns_msg_bimanual *msg = (struct sns_msg_bimanual*) buf;
  // Read
  int n_dofs, n_steps_left, n_steps_right, mode;
  n_dofs = msg->n_dof;

  // Left
  n_steps_left = msg->n_steps_left;
  n_steps_right = msg->n_steps_right;
  mode = msg->mode;

  // LEFT POS
  if( mode == 0 ) {

    // Hack: If closing (left_pos < 100.0), go smaller
    left_pos = msg->x[0]; 
    if( left_pos < 95.0 ) { left_pos *= 0.25; }
  } 
  // RIGHT POS
  else if( mode == 1 ) {
    right_pos = msg->x[0];
  } 

  return Py_BuildValue("(fif)", left_pos, mode, right_pos );
}



/**
 * @function getMsg
 * @brief Get serialized version of msg of trajectory
 */
std::string getMsg( const std::list<Eigen::VectorXd> &_path, int _n_dofs, double _maxVel, double _maxAccel, double _freq ) {

  if( _path.size() == 0 ) { std::string s; return s; }

  // Create a trajectory
  Eigen::VectorXd maxVel( _n_dofs );
  Eigen::VectorXd maxAccel( _n_dofs );

  for( int i = 0; i < _n_dofs; ++i ) {
    maxVel(i) = _maxVel;
    maxAccel(i) = _maxAccel;
  }

  Trajectory trajectory( Path(_path, 0.1), 
			 maxVel, maxAccel );

  trajectory.outputPhasePlaneTrajectory();
  if( !trajectory.isValid() ) {
    printf("[ERROR] Trajectory is not valid! \n");
  }
    
  // Create a joint trajectory msg   
  trajectory_msgs::JointTrajectory jt;

  double duration = trajectory.getDuration();
  double tn = 0;
  double dt = 1.0 / _freq;
  Eigen::VectorXd vel_t;
  Eigen::VectorXd pos_t;

  while( tn < duration ) {
    vel_t = trajectory.getVelocity(tn);
    pos_t = trajectory.getPosition(tn);

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize( _n_dofs );
    for( int i = 0; i < _n_dofs; ++i ) {
      p.positions[i] = pos_t(i);
    }
    p.time_from_start = ros::Duration(tn);
    jt.points.push_back(p);
    tn += dt;
  }

  // Send it over
  size_t serial_size = ros::serialization::serializationLength(jt);
  boost::shared_array<uint8_t> buffer( new uint8_t[serial_size] );
  ros::serialization::OStream stream(  buffer.get(), serial_size );
  ros::serialization::serialize( stream, jt );
  std::string str_msg;
  str_msg.reserve( serial_size );
  for( size_t i = 0; i < serial_size; ++i ) {
    str_msg.push_back( buffer[i] );
  }

  return str_msg;
}

/**
 * @function sendArmStates
 * @brief Send arm state (position and velocity through channel)
 * @brief Arguments: positions arrays, velocities array, channel_name
 */
static PyObject* sendArmStates( PyObject* self, PyObject* args ) {

  PyObject *q_obj, *dq_obj;
  Py_buffer q_view, dq_view;
  const char* chan_name;

  if( !PyArg_ParseTuple( args, "OOs", &q_obj, &dq_obj, &chan_name ) ) {
    return NULL;
  }

  if( PyObject_GetBuffer( q_obj, &q_view, PyBUF_ANY_CONTIGUOUS | PyBUF_FORMAT ) == -1 ||
      PyObject_GetBuffer( dq_obj, &dq_view, PyBUF_ANY_CONTIGUOUS | PyBUF_FORMAT ) == -1 ) {
    return NULL;
  }

  // Check size
  if( q_view.shape[0] != dq_view.shape[0] ) {
    printf("ERROR: Position and velocity have different sizes! \n");
    return NULL;
  }

  int N =  q_view.shape[0];

  // Send state message
  ach_channel_t state_chan;
  enum ach_status r;
  
  // Open / start the communication
  r = ach_open( &state_chan, chan_name, NULL );  
  if( r != ACH_OK ) { 
    printf("ERROR: Opening %s - code: %s errno: %s \n", chan_name, ach_result_to_string(r), strerror(errno)  ); 
    return NULL; 
  }

  sns_msg_motor_state* msg = sns_msg_motor_state_local_alloc(N);
  msg->mode = SNS_MOTOR_MODE_VEL; // HOW TO READ THIS?
  msg->header.n = N;


  for( int i = 0; i < N; ++i ) {
    msg->X[i].pos = ((double*)q_view.buf)[i];
    msg->X[i].vel = ((double*)dq_view.buf)[i];
  }
  
  r = ach_put( &state_chan, msg, sns_msg_motor_state_size_n((uint32_t)N) );
  
  ach_close( &state_chan );
  return Py_BuildValue("i", r );
}

/**
 * @function sendHandStates
 * @brief Send hand state (position and velocity through channel)
 * @brief Arguments: position of gripper, velocities array, channel_name
 */
static PyObject* sendHandStates( PyObject* self, PyObject* args ) {

  float q,dq;
  const char* chan_name;

  if( !PyArg_ParseTuple( args, "ffs", &q, &dq, &chan_name ) ) {
    return NULL;
  }

  int N =  1;

  // Send state message
  ach_channel_t state_chan;
  enum ach_status r;
  
  // Open / start the communication
  r = ach_open( &state_chan, chan_name, NULL );  
  if( r != ACH_OK ) { 
    printf("ERROR: Opening %s - code: %s errno: %s \n", chan_name, ach_result_to_string(r), strerror(errno)  ); 
    return NULL; 
  }
  //printf("Sending %s: %f and %f \n", chan_name, q, dq);
  sns_msg_motor_state* msg = sns_msg_motor_state_local_alloc(N);
  msg->mode = SNS_MOTOR_MODE_POS; // HOW TO READ THIS?
  msg->header.n = N;


  for( int i = 0; i < N; ++i ) {
    msg->X[i].pos = q;
    msg->X[i].vel = dq;
  }
  
  r = ach_put( &state_chan, msg, sns_msg_motor_state_size_n((uint32_t)N) );
  
  ach_close( &state_chan );
  return Py_BuildValue("i", r );
}


// Declare the methods
static PyMethodDef InterloperMethods[] = {
  {"sendArmStates", sendArmStates, METH_VARARGS, "Send motor states to simulator"},
  {"sendHandStates", sendHandStates, METH_VARARGS, "Send hand states to simulator"},
  {"readGatekeeperMsg", readGatekeeperMsg, METH_VARARGS, "Read gatekeeper messages"},
  {"readGatekeeperHandMsg", readGatekeeperHandMsg, METH_VARARGS, "Read gatekeeper hand messages"},
  {"sendGatekeeperMsg", sendGatekeeperMsg, METH_VARARGS, "Send msg to planner letting it know how the traj went"},
  {NULL,NULL,0,NULL}
};


// Initialize module
PyMODINIT_FUNC
initinterloper(void) {
  (void)Py_InitModule("interloper", InterloperMethods );
}



