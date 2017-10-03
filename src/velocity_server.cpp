#include "ros/ros.h"
#include "epos2/Velocity.h"

#include "wrap.h"

bool moveWithVelocity(epos2::Velocity::Request &req, epos2::Velocity::Response &res)
{
	unsigned int ulErrorCode = 0;
	// the force transform of the data type can cause problem
	ROS_INFO("request: targetVelocity=%ld", (long int)req.targetVelocity);
	MoveWithVelocity(g_pKeyHandle, g_usNodeId, (long)req.targetVelocity, &ulErrorCode);

	// short current;
	// int position_new;
	// ROS_INFO("now read position");
	// get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);
	// // res.current = current;
	// res.position_new = position_new;
	// sleep(0.01);
	ROS_INFO("now return");
	return true;
}

int main(int argc, char **argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	// Set parameter for usb IO operation
	SetDefaultParameters();
	// open device
	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}
	
	SetEnableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId, &ulErrorCode);

	ros::init(argc, argv, "epos2");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("moveWithVelocity", moveWithVelocity);

	ROS_INFO("Ready to move.");
	ros::spin();

	//disable epos
	SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	//close device
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return 0;
}