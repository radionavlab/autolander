#pragma once
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mg_msgs/WifiStatus.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <string>
#include <iostream>

class autolander
{
public:
	autolander(ros::NodeHandle &nh);
	void statusCallback(const mg_msgs::WifiStatus::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);
	void thrRepeatCallback(const std_msgs::Float64::ConstPtr &msg);
	void attRepeatCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
	double yaw_, twLand_, autoDisarmThreshold_;
	ros::Publisher thrustPub_, attSetPub_;
	ros::Subscriber gpsSub_, joySub_, imuSub_, statusSub_, throttleSub_, attThrottleSub_;
	ros::Timer timerPub_;
	bool isDisarmed_, hasNotLanded_, wifiIsGreen_;	

};

