#include "calibCoreAlgorithm.h"

#include <math.h>

#include <iostream>

using namespace cv;
using namespace std;

#define fisheye_calib

int main( int argc, char** argv )
{

#ifdef fisheye_calib

    //step-1:left cam calib
    calibCoreAlgorithm camLeft;
    camLeft._camera._cameraL=true;
    camLeft._configPath="../config/config.yaml";

    camLeft.doSingleCalibrateFishEye();

    cout<<"calib is ok... "<<endl;

#endif

    return true;
}
