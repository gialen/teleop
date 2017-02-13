#include <ros/ros.h>
#include <ros_myo/EmgArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Eigen>
// ROS cuustom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

# define PI 3.14159


class Extended_Madgw
{
public:
  Extended_Madgw();
  ~Extended_Madgw();


  void run();
  Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);


  double dt;



private:

  void callback_1(const sensor_msgs::Imu::ConstPtr& msg);
  void callback_2(const sensor_msgs::Imu::ConstPtr& msg);
  
  //tutte le acc provenienti dalle imu (NON MYO)
  void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg);
  void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg);


  //!!!!!!!!!!!callback che leggono le posizioni della kinect!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void callback_elbow(const geometry_msgs::Quaternion::ConstPtr& msg);
  void callback_wrist(const geometry_msgs::Quaternion::ConstPtr& msg);
  void callback_palm(const geometry_msgs::Quaternion::ConstPtr& msg);


  //<<<<<<<<<<<<<<<<<<<<<flag disconnec myo<<<<<<<<<<<<<<<<<<<<<<<<<
  void callback_disc_myo(const std_msgs::Empty::ConstPtr& msg);


  bool flag_run1_,flag_run2_, flag_run3_, flag_run4_, flag_run5_, flag_run6_, flag_run7_, flag_disc_;
  int ID;

  ros::NodeHandle n_; 

  std_msgs::Empty empty;

  //!!!!!!!!!!!!!!!! right side!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ros::Publisher pub_1_;
  ros::Publisher pub_2_;
  ros::Publisher pub_3_;
  ros::Subscriber sub_1_, sub_2_;
  ros::Subscriber sub_k_1_, sub_k_2_, sub_k_3_;

  
  Eigen::Vector3d acc_1_, gyro_1_;
  Eigen::Vector3d acc_2_, gyro_2_;
  Eigen::Vector3d acc_3_, gyro_3_;

  Eigen::Quaterniond q_old_1_;
  Eigen::Quaterniond q_old_2_;
  Eigen::Quaterniond q_old_3_;

  Eigen::Quaterniond q_est_1_;
  Eigen::Quaterniond q_est_2_;
  Eigen::Quaterniond q_est_3_;

  geometry_msgs::Quaternion q_1Link_;
  geometry_msgs::Quaternion q_2Link_;
  geometry_msgs::Quaternion q_3Link_;

  // [state x y z]
  Eigen::Vector4d shoulder_;
  Eigen::Vector4d elbow_;
  Eigen::Vector4d wrist_;
  Eigen::Vector4d palm_;




  //!!!!!!!!!!!!!!!!!!!common side-------> IMU NON MYO!!!!!!!!!!!!!!!!!!!
  ros::Subscriber sub_3acc_, sub_3gyro_;

  //>>>>>>>>>>>>>>>>>>>flag myo disconnect<<<<<<<<<<<<<<<<<<<<<<<
  ros::Subscriber sub_disc1_myo_, sub_disc2_myo_;
  ros::Publisher pub_flag_disc_;


};//End of class SubscribeAndPublish