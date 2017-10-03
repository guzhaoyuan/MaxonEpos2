/**
    time_test.cpp
    Purpose: check time function of time.h and ros.h

    @author Zhaoyuan Gu
    @version 0.1 08/23/17 
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "ros/ros.h"

using namespace std;

static long get_nanos(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (long)ts.tv_sec * 1000000000L + ts.tv_nsec;
}

int main(int argc, char **argv) {
    long nanos;
    long last_nanos;
    long start;
    nanos = get_nanos();
    last_nanos = nanos;
    start = nanos;
    // while (1) {
    //     nanos = get_nanos();
    //     // if (nanos - last_nanos > 100000000L) {
    //         printf("current nanos: %ld\n", nanos - start);
    //     //     last_nanos = nanos;
    //     // }
    // }
    
    ros::init(argc, argv, "epos2");
    ros::NodeHandle n;

    ros::Duration interval(0,25000000);

    cout<<interval.toSec()<<endl;
    return EXIT_SUCCESS;
}