#include <ros/ros.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mg_msgs/wifiStatus.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <string>
#include <iostream>

class autolander
{
public:
	autolander(ros::nodeHandle &nh);
	void statusCallback(const mg_msgs::wifiStatus::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
	void timerCallback(const ros::TimerEvent &event);

private:
	double yaw_, twLand_, autoDisarmThreshold_;
	ros::Publisher thrustPub_, attSetPub_;
	ros::Subscriber gpsSub_, joySub_, imuSub), statusSub_;
	ros::Timer timerPub_;
	bool isDisarmed_, hasNotLanded_, wifiIsGreen_;	

};
