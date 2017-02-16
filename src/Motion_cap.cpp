#include <Motion_cap.h>

Motion_cap::Motion_cap()
{
	std::string q_Arm, q_For, q_Palm;
	
	//rototraslazione da vito ancor a spalla
	std::vector<double> vec_Rvs, vec_Pvs;
	vec_Rvs.resize(9);
	vec_Pvs.resize(3);


	n_.getParam("Q_1_link", q_Arm);
	n_.getParam("Q_2_link", q_For);
	n_.getParam("Q_3_link", q_Palm);
	n_.getParam("Rvs", vec_Rvs);
	n_.getParam("Pvs", vec_Pvs);

    

    //Topic you want to subscribe
    sub_q_Arm_ = n_.subscribe(q_Arm, 10, &Motion_cap::callback_q_Arm, this);
    sub_q_For_ = n_.subscribe(q_For, 10, &Motion_cap::callback_q_For, this);
    sub_q_Palm_ = n_.subscribe(q_Palm, 10, &Motion_cap::callback_q_Palm, this);

    flag_qrifArm_ = flag_qrifFor_ = flag_qrifPalm_ = true;
    Lb_ = La_ = 3;
    Lm_ = 1;

    // scale_ee_ = 1.0 / 7;
    // scale_elbow_ = 0.4 / 3;
    scale_ee_ = 1 / 10;
    scale_elbow_ = 1 / 10;


    Rvs_ <<  vec_Rvs[0], vec_Rvs[1], vec_Rvs[2],
    			vec_Rvs[3], vec_Rvs[4], vec_Rvs[5],
    			vec_Rvs[6], vec_Rvs[7],vec_Rvs[8];

    // std::cout<<"RVS"<<Rvs_<<std::endl;

    Pvs_ << vec_Pvs[0], vec_Pvs[1], vec_Pvs[2];

}


Motion_cap::~Motion_cap()
{

}


void Motion_cap::Init()
{
	if(VR_sink)
	{
		std::string q0, q_Arm2For, q_For2Palm, Elbow_pos, Wrist_pos, Palm_pos;

		n_.getParam("q0", q0);
		n_.getParam("q_Arm2For", q_Arm2For);
		n_.getParam("q_For2Palm", q_For2Palm);
		n_.getParam("Elbow_pos", Elbow_pos);
		n_.getParam("Wrist_pos", Wrist_pos);
		n_.getParam("Palm_pos", Palm_pos);

	 	pub_q0_ = n_.advertise<geometry_msgs::Quaternion>(q0, 10);
	 	pub_q_Arm2For_ = n_.advertise<geometry_msgs::Quaternion>(q_Arm2For, 10);
	 	pub_q_For2Palm_ = n_.advertise<geometry_msgs::Quaternion>(q_For2Palm, 10);
	 	pub_Elbow_ = n_.advertise<geometry_msgs::Vector3>(Elbow_pos, 10);
	 	pub_Wrist_ = n_.advertise<geometry_msgs::Vector3>(Wrist_pos, 10);
	 	pub_Palm_ = n_.advertise<geometry_msgs::Vector3>(Palm_pos, 10);
	}
	else
	{

		std::vector<double> offset_elbow, q_pre, q_nxt;
		offset_elbow.resize(3);
		q_pre.resize(4);
		q_nxt.resize(4);

		n_.getParam("offset_elbow", offset_elbow);
		n_.getParam("q_pre", q_pre);
		n_.getParam("q_nxt", q_nxt);

		offset_elbow_ << offset_elbow[0],offset_elbow[1],offset_elbow[2];

		std::string command1, command2;

		q_pre_.w() = q_pre[0];
		q_pre_.vec() << q_pre[1], q_pre[2], q_pre[3];
		q_nxt_.w() = q_nxt[0];
		q_nxt_.vec() << q_nxt[1], q_nxt[2], q_nxt[3];


		if(Simulation)
		{
			n_.getParam("controller_cmd2", command2);
			n_.getParam("controller_cmd1", command1);

			pub_kuka_Elbow_ = n_.advertise<geometry_msgs::Pose>(command2, 10);
			pub_kuka_Palm_ = n_.advertise<geometry_msgs::Pose>(command1, 10);
		}
		else
		{
			n_.getParam("controller_effort_cmd2", command2);
			n_.getParam("controller_effort_cmd1", command1);

			pub_kuka_Elbow_ = n_.advertise<geometry_msgs::Pose>(command2, 10);
			pub_kuka_Palm_ = n_.advertise<geometry_msgs::Pose>(command1, 10);
		}
		
		//topic del kuka
	}
	// std::cout<<"VR"<< (VR_sink == true ? "true" : " false") <<std::endl;
	// std::cout<<"sim"<<(Simulation == true ? "true" : " false")<<std::endl;

}


void Motion_cap::callback_q_Arm(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	quat_Arm_.w() = msg->w;
	quat_Arm_.x() = msg->x;
	quat_Arm_.y() = msg->y;
	quat_Arm_.z() = msg->z;
	
	if(flag_qrifArm_)
	{
		quat_rif_Arm_ = quat_Arm_;
		flag_qrifArm_ = false;
	}


	// flag_run1_= true;

}

void Motion_cap::callback_q_For(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	quat_For_.w() = msg->w;
	quat_For_.x() = msg->x;
	quat_For_.y() = msg->y;
	quat_For_.z() = msg->z;

	if(flag_qrifFor_)
	{
		quat_rif_For_ = quat_For_;
		flag_qrifFor_ = false;
	}
	
	// flag_run1_= true;

}

void Motion_cap::callback_q_Palm(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	quat_Palm_.w() = msg->w;
	quat_Palm_.x() = msg->x;
	quat_Palm_.y() = msg->y;
	quat_Palm_.z() = msg->z;

	if(flag_qrifPalm_)
	{
		quat_rif_Palm_ = quat_Palm_;
		flag_qrifPalm_ = false;
	}
	
	// flag_run1_= true;

}


void Motion_cap::run()
{

	q0_ =  quat_rif_Arm_.inverse() * quat_Arm_;
	q01_ =  quat_rif_For_.inverse() * quat_For_;
	q02_ =  quat_rif_Palm_.inverse() * quat_Palm_;

	q_Arm2For_ = q0_.inverse() * q01_;
	q_For2Palm_ = q01_.inverse() * q02_;

	Rsb = q0_;
	Rba = q_Arm2For_;
	Ram = q_For2Palm_;




	T_rot_sb_.block<3,3>(0,0) = Rsb;
  	T_rot_sb_.block<3,1>(0,3) = Eigen::Vector3d::Zero();
  	T_rot_sb_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_rot_sb_(3,3) = 1;

  	T_tr_sb_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  	T_tr_sb_(0,3) = -Lb_;
  	T_tr_sb_.block<2,1>(1,3) = Eigen::Vector2d::Zero();
  	T_tr_sb_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_tr_sb_(3,3) = 1;

  	T_rot_ba_.block<3,3>(0,0) = Rba;
  	T_rot_ba_.block<3,1>(0,3) = Eigen::Vector3d::Zero();
  	T_rot_ba_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_rot_ba_(3,3) = 1;

  	T_tr_ba_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  	T_tr_ba_(0,3) = -La_;
  	T_tr_ba_.block<2,1>(1,3) = Eigen::Vector2d::Zero();
  	T_tr_ba_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_tr_ba_(3,3) = 1;

  	T_rot_am_.block<3,3>(0,0) = Ram;
  	T_rot_am_.block<3,1>(0,3) = Eigen::Vector3d::Zero();
  	T_rot_am_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_rot_am_(3,3) = 1;

  	T_tr_am_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  	T_tr_am_(0,3) = -Lm_;
  	T_tr_am_.block<2,1>(1,3) = Eigen::Vector2d::Zero();
  	T_tr_am_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  	T_tr_am_(3,3) = 1;

	//<<<<<<<<<<<matrice rototraslazione spalla braccio
	T_sb_ = T_rot_sb_ * T_tr_sb_;
	//<<<<<<<<<<<matrice rototraslazione  braccio avambraccio
	T_sa_ = T_sb_ * T_rot_ba_ * T_tr_ba_;
	//<<<<<<<<<<<matrice rototraslazione  avambraccio mano
	T_sm_ = T_sa_ * T_rot_am_ * T_tr_am_;

	if(VR_sink)
	{
		// Queste posizioni sono calcolate rispetto ad un sistema inerziale ottenuto dal sensor frame myo all'istante iniziale
		Elbow_pos_ = T_sb_.block<3,1>(0,3);
		Wrist_pos_ = T_sa_.block<3,1>(0,3);
		Palm_pos_ = T_sm_.block<3,1>(0,3);

		//Converto si geometry msg
		q0_pub.w = q0_.w();
		q0_pub.x = q0_.x();
		q0_pub.y = q0_.y();
		q0_pub.z = q0_.z();

		q_Arm2For_pub.w = q_Arm2For_.w();
		q_Arm2For_pub.x = q_Arm2For_.x();
		q_Arm2For_pub.y = q_Arm2For_.y();
		q_Arm2For_pub.z = q_Arm2For_.z();

		q_For2Palm_pub.w = q_For2Palm_.w();
		q_For2Palm_pub.x = q_For2Palm_.x();
		q_For2Palm_pub.y = q_For2Palm_.y();
		q_For2Palm_pub.z = q_For2Palm_.z();

		
		Elbow_pos_pub.x = Elbow_pos_.x();
		Elbow_pos_pub.y = Elbow_pos_.y();
		Elbow_pos_pub.z = Elbow_pos_.z();

		Wrist_pos_pub.x = Wrist_pos_.x();
		Wrist_pos_pub.y = Wrist_pos_.y();
		Wrist_pos_pub.z = Wrist_pos_.z();
		
		Palm_pos_pub.x = Palm_pos_.x();
		Palm_pos_pub.y = Palm_pos_.y();
		Palm_pos_pub.z = Palm_pos_.z();
		

		//Publish
		pub_q0_.publish(q0_pub);
		pub_q_Arm2For_.publish(q_Arm2For_pub);
		pub_q_For2Palm_.publish(q_For2Palm_pub);
		pub_Elbow_.publish(Elbow_pos_pub);
	 	pub_Wrist_.publish(Wrist_pos_pub);
	 	pub_Palm_.publish(Palm_pos_pub);
	}
	else
	{
		Elbow_pos_ = (Rvs_ * (T_sb_.block<3,1>(0,3) * scale_elbow_)) + Pvs_ + offset_elbow_;
		
		Palm_pos_ = (Rvs_ * (T_sm_.block<3,1>(0,3) * scale_ee_)) + Pvs_;

		kuka_q_ = q_pre_* q02_ * q_nxt_;

		kuka_elbow_.position.x = Elbow_pos_.x();
		kuka_elbow_.position.y = Elbow_pos_.y();
		kuka_elbow_.position.z = Elbow_pos_.z();

		kuka_ee_.position.x = Palm_pos_.x();
		kuka_ee_.position.y = Palm_pos_.y();
		kuka_ee_.position.z = Palm_pos_.z();

		kuka_ee_.orientation.w = kuka_q_.w();
		kuka_ee_.orientation.x = kuka_q_.x();
		kuka_ee_.orientation.y = kuka_q_.y();
		kuka_ee_.orientation.z = kuka_q_.z();

	

		pub_kuka_Elbow_.publish(kuka_elbow_);
		pub_kuka_Palm_.publish(kuka_ee_);

	}


}