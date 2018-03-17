#include <stdio.h>
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

    /** **/
	 int i,j;
    /*
    double a_data[] = { 0.18, 0.60, 0.57, 0.96,
			0.41, 0.24, 0.99, 0.58,
			0.14, 0.30, 0.97, 0.66,
			0.51, 0.13, 0.19, 0.85 };
    */
    double a[] = { 0.18, 0.60, 0.57, 0.96,
		   0.41, 0.24, 0.99, 0.58,
		   0.14, 0.30, 0.97, 0.66,
		   0.14, 0.30, 0.97, 0.66 };
    
    double *u = matrix_get(4,4);
    double *s = vector_get(4);
    double *v = matrix_get(4,4);
    
    matrix_svd( 4, 4, a, u, s, v );
    
    //matrix_print( 4, 4, u );
    //vector_print( 4, s );
    //matrix_print( 4, 4, v );
    
    //matrix_pinv( 4,4, a, a );
    
    //matrix_print(4,4,a);
    
    /** **/
	
  int count = 0;
  while (ros::ok())
  {
   
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
