/**
    test_math.cpp
    Purpose: check math function using sin

    @author Zhaoyuan Gu
    @version 0.1 08/23/17 
*/
#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>

#include <math.h>
#include <algorithm>    // std::max

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

#define position_offset 1000 
#define pulse_per_round 2000
#define PI 3.14159
#define V_LOW -8.0f
#define V_HIGH 8.0f

using namespace std;

int main(int argc, char **argv)
{
    int position = 0;
    float angle_old = (float)((position) % 4);///pulse_per_round*2*PI;
    cout<<position<<angle_old<<endl;
    // while(1){
    //     position += 2; // constant angular velocity
    //     // int position_global = position+position_offset;
    //     // float percentage = (float)(position_global % pulse_per_round)/pulse_per_round;
    //     // float angle = percentage*2*PI;
    //     float angle = (float)((position+position_offset) % pulse_per_round)/pulse_per_round*2*PI;
        
    //     // float velocity = -100;
    //     float velocity = (angle - angle_old)/0.025;
    //     velocity = min(V_HIGH,max(V_LOW,velocity));
    //     angle_old = angle;
    //     float sin1 = sin(angle);
    //     float cos1 = cos(angle);

    //     cout<<sin1<<","<<cos1<<","<<velocity<<endl;
    //     // cout<<"sin:"<<sin(PI/2)<<endl;
    // }
    //-5.658

	return 0;
}