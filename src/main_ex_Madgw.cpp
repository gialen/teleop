
#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "Extended_Madgw.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 50;

  ros::init(argc, argv, "Extended_Madgw_node");
  
  Extended_Madgw Obj;
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);

  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}