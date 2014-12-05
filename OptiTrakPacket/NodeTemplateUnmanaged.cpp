// NodeTemplateUnmanaged.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#pragma comment(lib, "C:\\Users\\gbrill\\Source\\Repos\\ROSIndigo\\ROSIndigoDLL\\Debug\\ROSIndigoDLL.lib")

int _tmain(int argc, _TCHAR* argv[])
{



	ROS_INFO("Started..");

	ros::init(argc, argv, "<NODE_NAME>");
	ros::NodeHandle n;

	//ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 25);


	ros::Rate loop_rate(15);
	ros::Rate loop_rate_idle(1);

	std::string default_param;
	while (ros::ok())
	{
		

		ros::spinOnce();
		loop_rate.sleep();
		
		
	}




	return 0;
}

