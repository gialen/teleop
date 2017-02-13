#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "Motion_cap.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 50;

  ros::init(argc, argv, "Motion_cap_node");
  ros::NodeHandle nh;

  Motion_cap Obj;
  ros::Rate r(rateHZ);

  nh.getParam("V_R_sink", Obj.VR_sink);
  nh.getParam("simulation", Obj.Simulation);
  Obj.Init();

  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}