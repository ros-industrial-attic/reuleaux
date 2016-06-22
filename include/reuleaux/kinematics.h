#ifndef KINEMATICS
#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN

#define IK_VERSION 61
#include "mh5_ikfast.cpp"
#include <stdio.h>
#include <stdlib.h>
#include<vector>

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif


using namespace std;

namespace kinematics{

class Kinematics{
public:

     //kinematics();
     //~kinematics();
     
 float SIGN(float x);
 float NORM(float a, float b, float c, float d);  
    void getPoseFromFK( const std::vector<double>  joint_values,  std::vector<double>& pose);

    bool isIKSuccess(std::vector<double> pose);
};
}

#endif //KINEMATICS

