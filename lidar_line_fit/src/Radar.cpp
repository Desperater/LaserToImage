
#include"OpenRadar.h"
#include <iostream>
#include <cmath>
// #include <io.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
const int MAX_POINT_COUNT = 1200;
int Rho[MAX_POINT_COUNT] = {0}; 


int main(int argc,char** argv){

    ros::init(argc,argv,"line_fit");
    ros::NodeHandle nh;  
    OpenRadar openRadar(nh);
    
    while (ros::ok())
    {
        openRadar.start();
        ros::spin();
    }
    
    return 0;
}