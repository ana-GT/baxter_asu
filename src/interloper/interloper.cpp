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

#include <Eigen/Core>

#include <motion_control/msgs/bimanual_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/serialization.h>

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

  // Get trajectory
  int counter;
  std::list<Eigen::VectorXd> leftPath;
  if( mode == 0 ) {
    counter = 0;
    for( int i = 0; i < n_steps_left; ++i ) {
      Eigen::VectorXd p(n_dofs);
      for( int j = 0; j < n_dofs; ++j ) {
	p(j) = msg->x[counter];
	counter++;
      }
      leftPath.push_back( p );
    }
  } 

  // Get the data from left path
  for( std::list<Eigen::VectorXd>::iterator it = leftPath.begin();
       it != leftPath.end(); ++it ) {
    std::cout << (*it).transpose() << std::endl;
  }

  // Create a joint trajectory msg
  trajectory_msgs::JointTrajectory jt;
  for( std::list<Eigen::VectorXd>::iterator it = leftPath.begin();
       it != leftPath.end(); ++it ) {
   
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize( (*it).size() );
  
    for( int i = 0; i < (*it).size(); ++i ) {
      p.positions[i] = (*it)(i);
    }
   
    jt.points.push_back(p);
  }

  ros::serialization::Serializer<trajectory_msgs::JointTrajectory> ser;

  size_t serial_size = ros::serialization::serializationLength(jt);
  boost::shared_array<uint8_t> buffer( new uint8_t[serial_size] );
  ros::serialization::OStream stream(  buffer.get(), serial_size );
  ros::serialization::serialize( stream, jt );
  std::string str_msg;
  str_msg.reserve( serial_size );
  for( size_t i = 0; i < serial_size; ++i ) {
    str_msg.push_back( buffer[i] );
  }

  return Py_BuildValue("s#", str_msg.c_str(), str_msg.size() );
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

// Declare the methods
static PyMethodDef InterloperMethods[] = {
    {"sendArmStates", sendArmStates, METH_VARARGS, "Send motor states to simulator"},
    {"readGatekeeperMsg", readGatekeeperMsg, METH_VARARGS, "Read gatekeeper messages"},
    {NULL,NULL,0,NULL}
};


// Initialize module
PyMODINIT_FUNC
initinterloper(void) {
  (void)Py_InitModule("interloper", InterloperMethods );
}



