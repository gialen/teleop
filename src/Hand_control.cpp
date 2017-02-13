#include <Hand_control.h>

Hand_control::Hand_control()
{

  std::vector<double> S_nmf;
  std::string For_emg, Arm_emg, Hand_mov, Stiff;
  S_nmf.resize(24);

  n_.getParam("Snmf", S_nmf);
  n_.getParam("For_emg", For_emg);
  n_.getParam("Arm_emg", Arm_emg);
  n_.getParam("Hand_mov", Hand_mov);
  n_.getParam("Stiff", Stiff);


	//Topic you want to subscribe
    sub_emg_For_ = n_.subscribe(For_emg, 10, &Hand_control::callback_emg_For, this);
    sub_emg_Arm_ = n_.subscribe(Arm_emg, 10, &Hand_control::callback_emg_Arm, this);

	//Topic you want to spub
    pub_Hand_close_ = n_.advertise<std_msgs::Float64>(Hand_mov, 10);
    pub_Stiff_ = n_.advertise<std_msgs::Float64>(Stiff, 10);

    emg_For_ = Eigen::VectorXd::Zero(8);
    emg_Arm_ = Eigen::VectorXd::Zero(8);
    S_nmf_ = Eigen::MatrixXd::Zero(3,8);

    S_nmf_ << S_nmf[0], S_nmf[1], S_nmf[2], S_nmf[3],S_nmf[4],S_nmf[5],S_nmf[6],S_nmf[7],
    			S_nmf[8],S_nmf[9],S_nmf[10],S_nmf[11],S_nmf[12],S_nmf[13],S_nmf[14],S_nmf[15],
    			S_nmf[16],S_nmf[17],S_nmf[18],S_nmf[19],S_nmf[20],S_nmf[21],S_nmf[22],S_nmf[23];

          // std::cout<<"matrice"<<S_nmf_<<std::endl;

}


Hand_control::~Hand_control()
{

}

void Hand_control::callback_emg_For(const ros_myo::EmgArray::ConstPtr& msg )
{
  for(int i = 0; i <= 7; i++)
  {
  	emg_For_(i)= msg->data[i];
   // std::cout<< emg_(0)<<" "<<emg_(1)<<" "<<emg_(2)<<" "<<emg_(3)<<" "<<emg_(4)<<" "<<emg_(5)<<" "<<emg_(6)<<" "<<emg_(7)<<std::endl;

  }
  

}

void Hand_control::callback_emg_Arm(const ros_myo::EmgArray::ConstPtr& msg )
{
  for(int i = 0; i <= 7; i++)
  {
    emg_Arm_(i)= msg->data[i];
   // std::cout<< emg_(0)<<" "<<emg_(1)<<" "<<emg_(2)<<" "<<emg_(3)<<" "<<emg_(4)<<" "<<emg_(5)<<" "<<emg_(6)<<" "<<emg_(7)<<std::endl;

  }
  

}

void Hand_control::run()
{
	mov_ = S_nmf_ * emg_For_;

	pub_mov_.x = mov_(0);
	pub_mov_.y = mov_(1);
	pub_mov_.z = mov_(2);

  if(pub_mov_.z > 5)
  {
    Hand_close_.data = 1.0;  
  }
  else
  {
    Hand_close_.data = 0.0;
  }

  Stiff_.data =  (emg_Arm_.sum() / 8) / 500;

  if(Stiff_.data > 1)
  {
    Stiff_.data = 1;

  }




	pub_Hand_close_.publish(Hand_close_);
  pub_Stiff_.publish(Stiff_);
}