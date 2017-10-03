/**
    controller.cpp
    Purpose: used for control the pendulum hardware

    @author Zhaoyuan Gu
    @version 0.1 08/23/17 
*/
#include "ros/ros.h"
#include <signal.h>
#include "epos2/Torque2.h" // service file

#include "wrap.h" // the head of epos control functions

#define position_offset 1000 
#define pulse_per_round 2000
#define PI 3.14159
#define V_LOW -10.0f
#define V_HIGH 10.0f
#define CURRENT_MAX 2.0f
#define CURRENT_MIN -2.0f // res.torque = (-1,1)
#define TORQUE_AMP 2000 //torque applied = TORQUE_AMP * res.torque // 2000 is for max real
#define MAX_STEP 200
/**
	define clockwise is minus, conterclockwise is positive
	define theta is the angle from upward axis to pumdulum, range (-PI , PI]
	state:[cos(theta) sin(thata) velocity]
	velocity range[-8.0 8.0]

	a round is 2000 trigs from the encoder
	when setup init, the pumdulum is downward and the position downward is 0,
	so the angle theta = (position+1000)%2000*2*PI
	velocity = (theta - theta_old)/dt
	reward = -(theta^2 + 0.1*theta_dt^2 + 0.001*action^2)
*/

ros::Time begin;
// ros::Duration interval(1.0); // 1s
ros::Duration interval(0,33000000); // 0s,33ms
ros::Time next;

int position_old, position_new; // for calc velocity
float angle_old, angle_new, pVelocityIs, pVelocityIs_old, reward, torque, torque2;
short current;

int random_init(epos2::Torque2::Request &req, epos2::Torque2::Response &res){
	// need to update angle while random init
	// wait until position do not change
	int position_tmp = 0, velocity_tmp = 0, delta = 1;
	float angle_new;
	unsigned int ulErrorCode = 0;
	while(delta != 0 && ros::ok()){
		get_position(g_pKeyHandle, g_usNodeId, &position_tmp, &ulErrorCode);
		get_velocity(g_pKeyHandle, g_usNodeId, &velocity_tmp, &ulErrorCode);
		angle_new = (float)((position_new+position_offset) % pulse_per_round)/pulse_per_round*2*PI;
		if(angle_new > PI){
			angle_new -= 2*PI; // angle range (-PI , PI]
	    }else if(angle_new < -PI){
	    	angle_new += 2*PI; // angle range (-PI , PI]
	    }
		if(abs(angle_new) < PI/2 && abs(velocity_tmp) < 1){// make it difficult to start from top
			delta = 0;
		}
		if(abs(angle_new) > PI/2 && abs(velocity_tmp) < 6){// start from lower position, with a relative low speed
			delta = 0;
		}
		ros::Duration(0.5).sleep();// sleep for 0.4s then compare position again
	}
	res.state_new[0] = cos(angle_new);res.state_new[1] = sin(angle_new);res.state_new[2] = velocity_tmp;
	cout<<"init success, return first state"<<endl;
	return true;
}

int down_init(epos2::Torque2::Request &req, epos2::Torque2::Response &res){
	// need to update angle while random init
	// wait until position do not change
	int delta = 1, position_tmp, position_tmp_old = 0;
	unsigned int ulErrorCode = 0;
	while(delta != 0 && ros::ok()){
		get_position(g_pKeyHandle, g_usNodeId, &position_tmp, &ulErrorCode);
		delta = position_tmp - position_tmp_old;
		position_tmp_old = position_tmp;
		ros::Duration(0.2).sleep();// sleep for 0.1s then compare position again
	}
	// when move back to zero position, record position and set position offset
	float angle = PI;
	res.state_new[0] = cos(angle);res.state_new[1] = sin(angle);res.state_new[2] = 0;
	cout<<"init success, return first state"<<endl;
	return true;
}

int zero_init(epos2::Torque2::Request &req, epos2::Torque2::Response &res){
	// need to update angle while random init
	// wait until position do not change
	int position_tmp = 1;
	unsigned int ulErrorCode = 0;

	ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	MoveToPosition(g_pKeyHandle, g_usNodeId, 0, 1, &ulErrorCode);
	while(position_tmp != 0 && ros::ok()){
		get_position(g_pKeyHandle, g_usNodeId, &position_tmp, &ulErrorCode);
		ros::Duration(0.5).sleep();// sleep for 0.5s then compare position again
	}
	ActivateProfileCurrentMode(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	// when move back to zero position, record position and set position offset
	float angle = PI;
	res.state_new[0] = cos(angle);res.state_new[1] = sin(angle);res.state_new[2] = 0;
	cout<<"init success, return first state"<<endl;
	return true;
}

bool applyTorque(epos2::Torque2::Request &req, epos2::Torque2::Response &res)
{
	if(req.init == 1){
		random_init(req, res);
		return true;
	}else{
		unsigned int ulErrorCode = 0;


		// ROS_INFO("now read position n current");
		get_position(g_pKeyHandle, g_usNodeId, &position_new, &ulErrorCode);

		// get_current(g_pKeyHandle, g_usNodeId, &current, &ulErrorCode);
		// get_velocity(g_pKeyHandle, g_usNodeId, &pVelocityIs, &ulErrorCode);
		
		//calc velocity before angle, using continuous position n angle, rad/s
		pVelocityIs = (float)(position_new - position_old)/pulse_per_round*2*PI/interval.toSec();
		if(abs(pVelocityIs) > V_HIGH){
			torque = req.torque * V_HIGH / pVelocityIs;// this is a strange way, to constrain the velocity
		}else{
			torque = req.torque;
		}
		pVelocityIs = min(V_HIGH,max(V_LOW,pVelocityIs)); // soft limit speed
		//calc angle
		angle_new = (float)((position_new+position_offset) % pulse_per_round)/pulse_per_round*2*PI;
		if(angle_new > PI){
			angle_new -= 2*PI; // angle range (-PI , PI]
	    }else if(angle_new < -PI){
	    	angle_new += 2*PI; // angle range (-PI , PI]
	    }

	    //calc reward
	    torque = min(CURRENT_MAX,max(CURRENT_MIN, torque)); // soft limit torque
	    torque2 = req.torque2;
	    torque2 = min(CURRENT_MAX,max(CURRENT_MIN, torque2)); // soft limit torque
	    // if(torque==0 & torque2 ==0){
	    // 	cout<<"rest"<<endl;
	    // }
		reward = -(angle_old*angle_old + 0.01*pVelocityIs_old*pVelocityIs_old + 0.001*req.torque*req.torque);
		// cout<<position_old<<",\t"<<angle_old<<",\t"<<pVelocityIs_old<<",\t"<<req.torque<<",\t"<<reward<<endl;

		// res.current = current;
		// res.position_new = position_new;
		// res.velocity = pVelocityIs;
		res.reward = reward;
		res.state_new[0] = cos(angle_new);res.state_new[1] = sin(angle_new);res.state_new[2] = pVelocityIs;

		if(req.position >= MAX_STEP) res.done = true;
		//update stored position and angle
		position_old = position_new;
		angle_old = angle_new;
		pVelocityIs_old = pVelocityIs;

		// the force transform of the data type can cause problem
		while((next - ros::Time::now()).toSec()<0){
			next += interval;
			// ROS_INFO("");
		}
		(next - ros::Time::now()).sleep();

		// if((next - ros::Time::now()).toSec()<0){
		// 	next += interval;
		// 	if((next - ros::Time::now()).toSec()<0){
		// 		next = 
		// 	}
		// }
		// (next - ros::Time::now()).sleep();





		// use position as step
		// ROS_INFO("now write: step=%ld, torque=%f", (long int)req.position, TORQUE_AMP*torque);
		SetCurrentMust(g_pKeyHandle, g_usNodeId, TORQUE_AMP*torque, &ulErrorCode);
		SetCurrentMust(g_pKeyHandle2, g_usNodeId2, TORQUE_AMP*torque2, &ulErrorCode);
		// ROS_INFO("now return");
	}
	return true;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  ROS_INFO("server shutdown.");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "epos2_controller3");
	ros::NodeHandle n;

	begin = ros::Time::now();
	next = begin + interval;
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
	if((lResult = OpenDevice2(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}
	// node 1 for pro, node 2 for adv


	// clear fault
	VCS_ClearFault(g_pKeyHandle, g_usNodeId, &ulErrorCode); 
	VCS_ClearFault(g_pKeyHandle2, g_usNodeId2, &ulErrorCode); 
	SetEnableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	SetEnableState(g_pKeyHandle2, g_usNodeId2, &ulErrorCode);
	ActivateProfileCurrentMode(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	ActivateProfileCurrentMode(g_pKeyHandle2, g_usNodeId2, &ulErrorCode);

	ros::ServiceServer service = n.advertiseService("applyTorque3", applyTorque);

	signal(SIGINT, mySigintHandler);

	ROS_INFO("Ready to move.");
	ros::spin();

	//disable epos
	SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	SetDisableState(g_pKeyHandle2, g_usNodeId2, &ulErrorCode);
	//close device
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	if((lResult = CloseDevice2(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return 0;
}