/**
 * @file view_kinect_node.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @function grabImage
 */
void grabImage( const sensor_msgs::Image::ConstPtr &_msg ) {
  
  uchar* data = (uchar*)&_msg->data[0];
  cv::Mat img( _msg->height, _msg->width, 
	       CV_8UC3,
	       data,
	       _msg->step );
  cv::imshow("rgb", img );
  cv::waitKey(30);

}

/**
 * @function grabDepth
 */
void grabDepth( const sensor_msgs::Image::ConstPtr &_msg ) {
  
  void* m = (void*)&_msg->data[0];
  float* toCopyFrom = (float*)(m);
  
  int index = 0;
  cv::Mat img( _msg->height, _msg->width, 
	       CV_32FC1 );
  cv::Mat showme( _msg->height, _msg->width, CV_8UC1 );

  for( int j = 0; j < _msg->height; ++j ) {
    for( int i = 0; i < _msg->width; ++i ) {
      float depth = toCopyFrom[index];
      index = index + 1;
      if( !isnan(depth) ) { img.at<float>(j,i) = depth; }
      else { img.at<float>(j,i) = 0; }

    }
  }

  double minP, maxP; cv::Point i1, i2;
  minMaxLoc( img, &minP, &maxP, &i1, &i2, cv::Mat() );
   printf("Min: %f max:%f \n", minP,maxP );

  float a = 255.0/(maxP-minP);
  float b = -255.0*minP/(maxP-minP);
  for( int j = 0; j < _msg->height; ++j ) {
    for( int i = 0; i < _msg->width; ++i ) {
      showme.at<uchar>(j,i) = (uchar)( a*img.at<float>(j,i) + b );
    }
  }
  
  cv::imshow("depth", showme );
  cv::waitKey(30);
}


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ros::init( argc, argv, "view_kinect_node" );
  ros::NodeHandle n;
  int rate;

  ros::Subscriber sub_rgb = n.subscribe( "/baxter/kinect/rgb/image_raw", 1000, grabImage );
  ros::Subscriber sub_depth = n.subscribe( "/baxter/kinect/depth/image_raw", 1000, grabDepth );
  ros::Rate r(rate);

  while( n.ok() ) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
