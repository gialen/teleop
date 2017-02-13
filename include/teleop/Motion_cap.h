#include <ros/ros.h>
#include <ros_myo/EmgArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
// ROS cuustom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

# define PI 3.14159

class Motion_cap
{
public:
  Motion_cap();
  ~Motion_cap();


  void run();
  void Init();
  bool VR_sink, Simulation;



private:

  void callback_q_Arm(const geometry_msgs::Quaternion::ConstPtr& msg);
  void callback_q_For(const geometry_msgs::Quaternion::ConstPtr& msg);
  void callback_q_Palm(const geometry_msgs::Quaternion::ConstPtr& msg);

  



  Eigen::Quaterniond quat_Arm_, quat_For_, quat_Palm_;
  Eigen::Quaterniond quat_rif_Arm_, quat_rif_For_, quat_rif_Palm_;
  Eigen::Quaterniond q_Arm2For_, q_For2Palm_;
  Eigen::Quaterniond q0_, q01_, q02_;
  Eigen::Quaterniond q_pre_, q_nxt_, kuka_q_;
  Eigen::Matrix3d Rsb, Rba, Ram;
  Eigen::Matrix4d T_rot_sb_, T_tr_sb_, T_rot_ba_, T_tr_ba_, T_rot_am_, T_tr_am_;
  Eigen::Matrix4d T_sb_, T_sa_, T_sm_;
  //roto e traslazione da vito ancor a shoulder 
  Eigen::Matrix3d Rvs_;
  Eigen::Vector3d Pvs_;
  Eigen::Vector3d offset_elbow_;


  Eigen::Vector3d Elbow_pos_, Wrist_pos_, Palm_pos_;



  geometry_msgs::Quaternion q0_pub, q_Arm2For_pub, q_For2Palm_pub;
  geometry_msgs::Vector3 Elbow_pos_pub, Wrist_pos_pub, Palm_pos_pub;
  geometry_msgs::Pose kuka_ee_, kuka_elbow_; 


  double Lb_, La_, Lm_;
  double scale_ee_, scale_elbow_;


  bool flag_qrifArm_, flag_qrifFor_, flag_qrifPalm_;


  
  ros::NodeHandle n_; 

  ros::Subscriber sub_q_Arm_, sub_q_For_, sub_q_Palm_;

  ros::Publisher pub_q0_, pub_q_Arm2For_, pub_q_For2Palm_;
  ros::Publisher pub_Elbow_, pub_Wrist_, pub_Palm_;
  ros::Publisher pub_kuka_Elbow_, pub_kuka_Palm_;

  



};//End of class SubscribeAndPublish