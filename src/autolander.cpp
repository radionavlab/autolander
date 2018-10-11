#include "autolander.hpp"
#include <ros/ros.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "autolander");
 	ros::NodeHandle nh;

  	try
  	{
		autolander lander(nh);
		ros::spin();
  	}
  	catch(const std::exception &e)
  	{
		ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
		return 1;
  	}
	return 0;

}


autolander::autolander(ros::NodeHandle &nh)
{
	double twTemp;
	std::string nodename, detectorTopic, thrTopic, attTopic, posRefTopic;
	nodename = ros::this_node::getName();
	ros::param::get(nodename + "/twMax",twTemp);
	ros::param::get(nodename + "/wifiTopic",detectorTopic);
	ros::param::get(nodename + "/disarmThresh",autoDisarmThreshold_);
	ros::param::get(nodename + "/throttleInputTopic",thrTopic);
	ros::param::get(nodename + "/attitudeInputTopic",attTopic);
	ros::param::get(nodename + "/positionReferenceTopic",posRefTopic);
	double pubrate(20.0);
	twLand_ = 0.9/twTemp;  //landing accelerates to 0.2gs
	isDisarmed_ = false;
	hasNotLanded_ = true;
	wifiIsGreen_ = true;
	gpsSub_ = nh.subscribe("local_odom", 1, &autolander::odomCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	joySub_ = nh.subscribe("joy", 1, &autolander::joyCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	imuSub_ = nh.subscribe("mavros/imu/data_raw", 1, &autolander::imuCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	statusSub_ = nh.subscribe(detectorTopic.c_str(), 1, &autolander::statusCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	throttleSub_ = nh.subscribe(thrTopic.c_str(), 1, &autolander::thrRepeatCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	attThrottleSub_ = nh.subscribe(attTopic.c_str(), 1, &autolander::attRepeatCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	posLocalSub_ = nh.subscribe(posRefTopic.c_str(), 1, &autolander::posRepeatCallback, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
	thrustPub_ = nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 1);
	attSetPub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude",1);
	posSetPub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	timerPub_ = nh.createTimer(ros::Duration(1.0/pubrate), &autolander::timerCallback, this, false);
}

//Callbacks
void autolander::thrRepeatCallback(const std_msgs::Float64::ConstPtr &msg)
{
	if(wifiIsGreen_)
	{
		thrustPub_.publish(*msg);
	}
}


void autolander::attRepeatCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if(wifiIsGreen_)
	{
		attSetPub_.publish(*msg);
	}
}


void autolander::posRepeatCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if(wifiIsGreen_)
	{
		posSetPub_.publish(*msg);
	}
}


//Listens to wifi topic
void autolander::statusCallback(const mg_msgs::PingStatus::ConstPtr &msg)
{
	std::string comparisonString("ground_ip");
	std::string interface = msg->host;
	if(interface.compare(comparisonString.c_str()) !=0 && !msg->alive)
	{wifiIsGreen_=false;}
}

//Joy is used to determine whether or not the quad is armed
void autolander::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	if(msg->buttons[0]==1)
	{isDisarmed_=true;}
}


//Shock upon landing used to determine status
void autolander::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	if(!wifiIsGreen_)
        {
            double accel;
	    accel = (msg->linear_acceleration.x * msg->linear_acceleration.x + 
			 msg->linear_acceleration.y * msg->linear_acceleration.y + 
			 msg->linear_acceleration.z * msg->linear_acceleration.z);
	    accel = sqrt(accel);

	    double vertAccel = msg->linear_acceleration.z;

	    //NOTE: imu specific force records g while stationary.  Collision with ground produces 0 force
	    //if(accel < autoDisarmThreshold_)
	    //{hasNotLanded_=false;}
	    if(vertAccel < -5)
	    {hasNotLanded_=false;}
        }
}


//Quad should maintain yaw to avoid any ugly spins
void autolander::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	//Extract yaw from quat
	Eigen::Matrix<double,4,1> quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	quat(1) = 0;
	quat(3) = 0;
	double len = quat(0)*quat(0)+quat(2)*quat(2);
	yaw_ = 2.0*acos(quat(0)/len);
}


//Timer publishes spoofed commands upon timeout
void autolander::timerCallback(const ros::TimerEvent &event)
{
        static int seq(0);
        seq++;
        /*if(seq>20*20)
        {
                ROS_INFO("SEQ TEST ENABLED");
                wifiIsGreen_=false;
        }*/
	if(!wifiIsGreen_ && !isDisarmed_)
	{
		if(hasNotLanded_)
		{
			std_msgs::Float64 thrustMsg;
			thrustMsg.data = twLand_;
			thrustPub_.publish(thrustMsg);
		}
		else
		{
			std_msgs::Float64 thrustMsg;
			thrustMsg.data = 0.0;
			thrustPub_.publish(thrustMsg);
		}

		//Set yaw as a constant value
		geometry_msgs::PoseStamped attMsg;
		attMsg.pose.orientation.w = cos(yaw_/2);
		attMsg.pose.orientation.x = 0.0;
		attMsg.pose.orientation.y = sin(yaw_/2);
		attMsg.pose.orientation.z = 0.0;
		attSetPub_.publish(attMsg);
	}
}
