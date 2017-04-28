#include <Extended_Madgw.h>

Extended_Madgw::Extended_Madgw()
{
	std::string pub_1, pub_2, pub_3, pub_flag_disc;
	std::string sub_1, sub_2, sub_k_1, sub_k_2, sub_k_3, sub_disc1_myo, sub_disc2_myo;
	double q_old_1_w, q_old_2_w, q_old_3_w;
	std::vector<double> q_old_1_vec, q_old_2_vec, q_old_3_vec;
    

	n_.getParam("Q_1_link", pub_1);
	n_.getParam("Q_2_link", pub_2);
	n_.getParam("Q_3_link", pub_3);
	n_.getParam("pub_flag_disc", pub_flag_disc);

    n_.getParam("Imu_1_link", sub_1);
    n_.getParam("Imu_2_link", sub_2);
    n_.getParam("sub_k_1", sub_k_1);
    n_.getParam("sub_k_2", sub_k_2);
    n_.getParam("sub_k_3", sub_k_3);
    n_.getParam("sub_disc1_myo", sub_disc1_myo);
    n_.getParam("sub_disc2_myo", sub_disc2_myo);
    
    n_.getParam("ID", ID);

    n_.getParam("q_old_1_w", q_old_1_w);
    n_.getParam("q_old_1_vec", q_old_1_vec);
    n_.getParam("q_old_2_w", q_old_2_w);
    n_.getParam("q_old_2_vec", q_old_2_vec);
    n_.getParam("q_old_3_w", q_old_3_w);
    n_.getParam("q_old_3_vec", q_old_3_vec);

    
	 //Topic you want to publish
    // pub del quaternione stimato relativo al primo link
    pub_1_ = n_.advertise<geometry_msgs::Quaternion>(pub_1, 10);
    // pub del quaternione stimato relativo al secondo link
    pub_2_ = n_.advertise<geometry_msgs::Quaternion>(pub_2, 10);
    // pub del quaternione stimato relativo al terzo link
    pub_3_ = n_.advertise<geometry_msgs::Quaternion>(pub_3, 10);
    pub_flag_disc_ = n_.advertise<std_msgs::Empty>(pub_flag_disc, 10);

    //Topic you want to subscribe
    //sub al topic Imu del primo link della catena (esempio: right_arm/Arm_r_imu)
    sub_1_ = n_.subscribe(sub_1, 10, &Extended_Madgw::callback_1, this);
    //sub al topic Imu del secondo link della catena (esempio: right_arm/For_r_imu)
    sub_2_ = n_.subscribe(sub_2, 10, &Extended_Madgw::callback_2, this);
    //sub al topic acc e gyro del secondo link della catena, dove non c'è il myo(la callback carica il parametro ID)
    sub_3acc_ = n_.subscribe("/qb_class_imu/acc", 10, &Extended_Madgw::callback_imu_acc, this);
    sub_3gyro_ = n_.subscribe("/qb_class_imu/gyro", 10, &Extended_Madgw::callback_imu_gyro, this);
    // sub al topic della kinect, primo link, secondo link e terzo link della catena
    sub_k_1_ = n_.subscribe(sub_k_1, 10, &Extended_Madgw::callback_elbow, this);
    sub_k_2_ = n_.subscribe(sub_k_2, 10, &Extended_Madgw::callback_wrist, this);
    sub_k_3_ = n_.subscribe(sub_k_3, 10, &Extended_Madgw::callback_palm, this);
    
	//sub Flag Myo disconnect
    sub_disc1_myo_ = n_.subscribe(sub_disc1_myo, 10, &Extended_Madgw::callback_disc_myo, this);
    sub_disc2_myo_ = n_.subscribe(sub_disc2_myo, 10, &Extended_Madgw::callback_disc_myo, this);



    // q_old_1_.w() = 0; 
    // q_old_1_.vec() << 0, 0.7071, -0.7071;    
    // q_old_2_.w() = 0; 
    // q_old_2_.vec() << 0, 0.7071, -0.7071;    
    // q_old_3_.w() = 0; 
    // q_old_3_.vec() << 0, 0.7071, -0.7071;

    q_old_1_.w() = q_old_1_w; 
    q_old_1_.vec() << q_old_1_vec[0], q_old_1_vec[1], q_old_1_vec[2];    
    q_old_2_.w() = q_old_2_w; 
    q_old_2_.vec() << q_old_2_vec[0], q_old_2_vec[1], q_old_2_vec[2];    
    q_old_3_.w() = q_old_3_w; 
    q_old_3_.vec() << q_old_3_vec[0], q_old_3_vec[1], q_old_3_vec[2];

    flag_run1_ = flag_run2_ = flag_run3_ = flag_run4_ = flag_run5_ = flag_run6_ = flag_run7_ = false;
    flag_disc_ = true;
    flag_init_ = false;

    n_sample_ = 100;
    step_=0;
	data_ = Eigen::MatrixXd::Zero(n_sample_,3);

    shoulder_(0) = 2;
    shoulder_(1) = 0;
    shoulder_(2) = 0;
    shoulder_(3) = 0;

}

Extended_Madgw::~Extended_Madgw()
{

}

void Extended_Madgw::callback_disc_myo(const std_msgs::Empty::ConstPtr& msg)
{	
	//non appena un myo della catena si disconnette il flag diventa false e il run() non stima più quaternione
	flag_disc_= false;
}


void Extended_Madgw::callback_1(const sensor_msgs::Imu::ConstPtr& msg)
{
	acc_1_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	gyro_1_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	
	flag_run1_= true;
}


void Extended_Madgw::callback_2(const sensor_msgs::Imu::ConstPtr& msg)
{
  	acc_2_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  	gyro_2_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  	
  	flag_run2_  = true;

}

//tutte le acc provenienti dalle imu (NON MYO)
void Extended_Madgw::callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
  	acc_3_ << msg->m[ID].x, msg->m[ID].y, msg->m[ID].z;
  	
  	flag_run3_  = true;
}

//tutte le vel angol provenienti dalle imu (NON MYO)
void Extended_Madgw::callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
	if(flag_init_)
	{
		gyro_3_ << (msg->m[ID].x - offset_(0)), (msg->m[ID].y  - offset_(1)), (msg->m[ID].z - offset_(2));
		// std::cout<<gyro_3_(0)<<" "<<gyro_3_(1)<<" "<<gyro_3_(2)<<std::endl;
		
	}
	else
	{
		gyro_3_ << msg->m[ID].x, msg->m[ID].y, msg->m[ID].z;
	}
  	

  	flag_run4_  = true;


}


//////////!!!!!!!!!!!callback che leggono le posizioni della kinect!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void Extended_Madgw::callback_elbow(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	elbow_ << msg->w, msg->x, msg->y, msg->z;
	
	flag_run5_= true;
}

void Extended_Madgw::callback_wrist(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	wrist_ << msg->w, msg->x, msg->y, msg->z;
	
	flag_run6_= true;
}

void Extended_Madgw::callback_palm(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	palm_ << msg->w, msg->x, msg->y, msg->z;
	
	flag_run7_= true;
}

Eigen::Quaterniond Extended_Madgw::madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt)
{

	double Beta = 0.035;
	int i;

	Eigen::VectorXd F(4);
	Eigen::MatrixXd J(4, 4);
	Eigen::VectorXd F_partial(3);
	Eigen::MatrixXd J_partial(3, 4);
	Eigen::Vector3d Acc_earth(0.0, 0.0, 1.0);
	Eigen::Vector3d kinect_vect;
	Eigen::VectorXd step(4);
	Eigen::Quaterniond q_step;
	Eigen::Quaterniond qDot;
	Eigen::Quaterniond gyro_q;
	Eigen::Quaterniond q_ang_rate;
	Eigen::Quaterniond q_nabla;
	Eigen::Quaterniond q_integr;
	Eigen::Quaterniond q_est;
	

	//Normalizzazione acc
	acc = acc / acc.norm();

	// for(i = 0; i < 3; i++)
	// {
	// 	if(abs(gyro(i)) < 5)
	// 	{
	// 		gyro(i) = 0;	
	// 	}
	// }
	// std::cout<<gyro(0)<<" "<<gyro(1)<<" "<<gyro(2)<<std::endl;
	
	//gradi to rad al sec
	gyro = gyro * (PI / 180);
	gyro_q.w() = 0;
	gyro_q.vec() = gyro;

	///////////////////////////////
	double q1 = q_old.w();
	double q2 = q_old.x();
	double q3 = q_old.y();
	double q4 = q_old.z();

	double d1x = Acc_earth(0);
	double d1y = Acc_earth(1);
	double d1z = Acc_earth(2);

	double s1x = acc(0);
	double s1y = acc(1);
	double s1z = acc(2);
	
	// ROS_INFO("I heard: [%f]", acc(0));

	if (State_joint_nxt == 2 && State_joint_prev == 2)
	{
		// kinect vector
		kinect_vect = (Joint_nxt - Joint_prev) / (Joint_nxt - Joint_prev).norm();

		// 
		double d2x = kinect_vect(0);
		double d2y = kinect_vect(1);
		double d2z = kinect_vect(2);


		F <<	2*d1x*(0.5 - q3*q3 - q4*q4) + 2*d1y*(q1*q4 + q2*q3) + 2*d1z*(q2*q4 - q1*q3) - s1x, 
				2*d1x*(q2*q3 - q1*q4) + 2*d1y*(0.5 - q2*q2 - q4*q4) + 2*d1z*(q1*q2 + q3*q4) - s1y,
				2*d1x*(q1*q3 - q2*q4) + 2*d1y*(q3*q4 - q1*q2) + 2*d1z*(0.5 - q2*q2 - q3*q3) - s1z,
			  // - q1*q1 - q2*q2 + q3*q3 + q4*q4 - d2x; 
			  - 2*q1*q4 - 2*q2*q3 - d2y;
				//2*q1*q3 - 2*q2*q4 - d2z;

		

		J <<  2*d1y*q4-2*d1z*q3  ,   2*d1y*q3+2*d1z*q4             ,     -4*d1x*q3+2*d1y*q2-2*d1z*q1      ,      -4*d1x*q4+2*d1y*q1+2*d1z*q2,  
	    	 -2*d1x*q4+2*d1z*q2  ,   2*d1x*q3-4*d1y*q2+2*d1z*q1    ,      2*d1x*q2+2*d1z*q4               ,      -2*d1x*q1-4*d1y*q4+2*d1z*q3,
	      	  2*d1x*q3-2*d1y*q2  ,   2*d1x*q4-2*d1y*q1-4*d1z*q2    ,      2*d1x*q1+2*d1y*q4-4*d1z*q3      ,                2*d1x*q2+2*d1y*q3,
	                      // -2*q1  ,                        -2*q2    ,                            2*q3      ,                             2*q4;
	                      -2*q4  ,                        -2*q3    ,                           -2*q2      ,                            -2*q1;
	                     //  2*q3  ,                        -2*q4    ,                            2*q1      ,                            -2*q2;

		step = J.transpose() * F;
	}
	else
	{
		F_partial <<	2*d1x*(0.5 - q3*q3 - q4*q4) + 2*d1y*(q1*q4 + q2*q3) + 2*d1z*(q2*q4 - q1*q3) - s1x, 
				2*d1x*(q2*q3 - q1*q4) + 2*d1y*(0.5 - q2*q2 - q4*q4) + 2*d1z*(q1*q2 + q3*q4) - s1y,
				2*d1x*(q1*q3 - q2*q4) + 2*d1y*(q3*q4 - q1*q2) + 2*d1z*(0.5 - q2*q2 - q3*q3) - s1z;
		

		J_partial <<  2*d1y*q4-2*d1z*q3  ,   2*d1y*q3+2*d1z*q4             ,     -4*d1x*q3+2*d1y*q2-2*d1z*q1      ,      -4*d1x*q4+2*d1y*q1+2*d1z*q2,  
	    	         -2*d1x*q4+2*d1z*q2  ,   2*d1x*q3-4*d1y*q2+2*d1z*q1    ,      2*d1x*q2+2*d1z*q4               ,      -2*d1x*q1-4*d1y*q4+2*d1z*q3,
	      	          2*d1x*q3-2*d1y*q2  ,   2*d1x*q4-2*d1y*q1-4*d1z*q2    ,      2*d1x*q1+2*d1y*q4-4*d1z*q3      ,                2*d1x*q2+2*d1y*q3;
	      
	    step = J_partial.transpose() * F_partial;               
	}


	step = step / step.norm();

	q_step.w() = step(0);
	q_step.x() = step(1);
	q_step.y() = step(2);
	q_step.z() = step(3);

	q_ang_rate = (q_old * gyro_q);
	q_ang_rate.w() = 0.5 * q_ang_rate.w();
	q_ang_rate.vec() = 0.5 * q_ang_rate.vec();

	q_nabla.w() = Beta * q_step.w();
	q_nabla.vec() = Beta * q_step.vec();

	//calcolo di qDot
	qDot.w() = q_ang_rate.w() - q_nabla.w();
	qDot.vec() = q_ang_rate.vec() - q_nabla.vec();

	///////////////////////////////////////
	q_integr.w() = qDot.w() * dt;
	q_integr.vec() = qDot.vec() * dt;

	q_est.w() = q_old.w() + q_integr.w();
	q_est.vec() = q_old.vec() + q_integr.vec();


	q_est.normalize();

	return q_est;
}



void Extended_Madgw::run()
{
	if(flag_run4_)
	{
			if(step_ < n_sample_)
			{
				data_(step_,0) = gyro_3_(0);
				data_(step_,1) = gyro_3_(1);
				data_(step_,2) = gyro_3_(2);
				step_++;
			}
			else
			{
				if(!flag_init_)
				{
					offset_(0) = data_.col(0).sum() / n_sample_;
					offset_(1) = data_.col(1).sum() / n_sample_;
					offset_(2) = data_.col(2).sum() / n_sample_;
					flag_init_ = true;
				}
				
			}
	}
	


	if(flag_run1_ && flag_run2_ && flag_run3_ && flag_run4_ && flag_run5_ && flag_run6_ && flag_run7_ && flag_disc_ && flag_init_)
	{

	q_est_1_ = madgwick_kin(acc_1_, gyro_1_, q_old_1_, shoulder_(0), shoulder_.tail<3>() , elbow_(0), elbow_.tail<3>());
	q_est_2_ = madgwick_kin(acc_2_, gyro_2_, q_old_2_, elbow_(0), elbow_.tail<3>() , wrist_(0), wrist_.tail<3>());
	q_est_3_ = madgwick_kin(acc_3_, gyro_3_, q_old_3_, wrist_(0), wrist_.tail<3>() , palm_(0), palm_.tail<3>());
	q_old_1_ = q_est_1_;
	q_old_2_ = q_est_2_;
	q_old_3_ = q_est_3_;

	q_1Link_.w = q_est_1_.w();
	q_1Link_.x = q_est_1_.x();
	q_1Link_.y = q_est_1_.y();
	q_1Link_.z = q_est_1_.z();

	q_2Link_.w = q_est_2_.w();
	q_2Link_.x = q_est_2_.x();
	q_2Link_.y = q_est_2_.y();
	q_2Link_.z = q_est_2_.z();

	q_3Link_.w = q_est_3_.w();
	q_3Link_.x = q_est_3_.x();
	q_3Link_.y = q_est_3_.y();
	q_3Link_.z = q_est_3_.z();

	pub_1_.publish(q_1Link_);
	pub_2_.publish(q_2Link_);
	pub_3_.publish(q_3Link_);
	}

	if(!flag_disc_)
	{
		pub_flag_disc_.publish(empty);
	}



	
	std::cout<< flag_run1_<<" "<< flag_run2_<<" "<< flag_run3_<<" "<< flag_run4_<<" "<< flag_run5_<<" "<< flag_run6_<<" "<< flag_run7_<<" " <<std::endl;
}


// void Extended_Madgw::init()
// {
	
	
    
// 	while(flag_init)
// 	{
// 		if(flag_run4_)
// 		{
// 			for(int i = 0; i < n_sample_; i++)
// 	    	{
// 		    	data_(i,0) = gyro_3_(0);
// 		    	data_(i,1) = gyro_3_(1);
// 		    	data_(i,2) = gyro_3_(2);
// 		    	usleep(dt * 1000000);
// 	   		}

// 		    offset_(0) = data_.col(0).sum() / n_sample_;
// 			offset_(1) = data_.col(1).sum() / n_sample_;
// 			offset_(2) = data_.col(2).sum() / n_sample_;
// 			flag_init = false;
// 		}
		
// 	}
    
// }