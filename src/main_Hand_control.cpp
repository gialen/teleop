#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "Hand_control.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 50;

  ros::init(argc, argv, "Hand_control_node");
  ros::NodeHandle nh;

  Hand_control Obj;
  ros::Rate r(rateHZ);

  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}