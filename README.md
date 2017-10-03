# MaxonEpos2

A controller written in CPP to control maxon DC motor,  support position, velocity & current control, communicate with controller using rospy

## Protocal

Torque.srv

	float64 position # used as step counter
	float64 torque
	int16 init # if true, the controller random init a state
	---
	float64[3] state_new # cos() sin() velocity
	float64 reward #used
	bool done # used
	float64 velocity # not used, calc by differenciate position
	float64 position_new #-pi to pi, now not used
	float64 current # not used

## EPOS2 info

using USB mode, no need to set baudrate

for EPOS2 setup for linux, see docs/EPOS Command Library.pdf

for driver and library, find 'EPOS_Linux_Library' on maxon official site(https://www.maxonmotorusa.com/maxon/view/product/control/Positionierung/347717)

- g_usNodeId = 1;
- g_deviceName = "EPOS2";
- g_protocolStackName = "MAXON SERIAL V2";
- g_interfaceName = "USB";
- g_portName = "USB0";

## Framework

client: request position or torque

server: receive request and execute, return new state and info, then wait for new request

## Files

### src

controller.cpp: node for controlling torque for servo1
controller2.cpp: node for controlling torque for servo2
controller3.cpp: node for controlling torque for servo1&2

test_epos2.cpp: test for develop
wrap.cpp: wrap api to make code clean and easy to program
time_test.cpp: test time

position_server.cpp: create service to control postion
current_server.cpp: create service to control current
velocity_server.cpp: create service to control velocity

### scripts

request.py: node for requesting service and get info from controller, now backup, not for use

position_client.py: request position control service
current_client.py: request current control service
velocity_client.py: request velocity control service

### include

headings

## Usages

use as a ros package, put the whole epos2 files under workspace/src and compile with catkin

	# under ${workspace}
	catkin_make
	rosrun epos2 current_server
	# in another terminal window
	rosrun epos2 current_client.py
	
## TODO

- try ros control