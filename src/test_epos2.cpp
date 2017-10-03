#include "wrap.h"

const string g_programName = "TEST";

int main(int argc, char** argv)
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
	// read position
	// if((lResult = get_position(g_pKeyHandle, g_usNodeId, &ulErrorCode))!=MMC_SUCCESS)
	// {
	// 	LogError("CloseDevice", lResult, ulErrorCode);
	// 	return lResult;
	// }

	//enable epos
	SetEnableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	//enable position mode
	ActivateProfileCurrentMode(g_pKeyHandle, g_usNodeId, &ulErrorCode);

	short targetCurrent = 20;
	SetCurrentMust(g_pKeyHandle, g_usNodeId, targetCurrent, &ulErrorCode);
	while(1)
	{
		sleep(0.1);
		get_current(g_pKeyHandle, g_usNodeId, &targetCurrent, &ulErrorCode);
	}
	// MoveWithVelocity(g_pKeyHandle, g_usNodeId, 80, &ulErrorCode);
	// get_TargetVelocity(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	// sleep(2);
	// get_velocity(g_pKeyHandle, g_usNodeId, &ulErrorCode);

	//disable epos
	// SetDisableState(g_pKeyHandle, g_usNodeId, &ulErrorCode);
	//close device
	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return lResult;
}



