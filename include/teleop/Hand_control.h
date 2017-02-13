#include <ros/ros.h>
#include <ros_myo/EmgArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>

class Hand_control
{
public:
  Hand_control();
  ~Hand_control();


  void run();
  


private:

  void callback_emg_For(const ros_myo::EmgArray::ConstPtr& msg);
	void callback_emg_Arm(const ros_myo::EmgArray::ConstPtr& msg);


  	ros::NodeHandle n_; 

  	ros::Subscriber sub_emg_For_, sub_emg_Arm_;
  	ros::Publisher pub_Hand_close_, pub_Stiff_;

  	Eigen::VectorXd emg_For_, emg_Arm_;
  	Eigen::MatrixXd S_nmf_;

  	Eigen::Vector3d mov_;

  	geometry_msgs::Vector3 pub_mov_;
    std_msgs::Float64 Hand_close_,  Stiff_;



};//End of class SubscribeAndPublish