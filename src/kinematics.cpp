#include <reuleaux/kinematics.h>

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif




using namespace std;
IKREAL_TYPE eerot[9],eetrans[3];
namespace kinematics{


float Kinematics::SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float Kinematics::NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


void Kinematics::getPoseFromFK(const std::vector<double> joint_values,  std::vector<double>& pose){

     #if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif
    IKREAL_TYPE joints[num_of_joints];
    
    //cout<<joint_values[2]<<endl;
    
    for (unsigned int i=0; i<num_of_joints; i++)
        {
	   
            joints[i] = joint_values[i];
            
        }
#if IK_VERSION > 54
    // for IKFast 56,61
        ComputeFk(joints, eetrans, eerot); // void return
#else
        // for IKFast 54
        fk(joints, eetrans, eerot); // void return
#endif
     //cout<<"translation: "<<eetrans[0]<<" "<<eetrans[1]<<" "<<eetrans[2]<<endl;
	// Convert rotation matrix to quaternion (Daisuke Miyazaki)
        float q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
        float q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
        float q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
        float q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
        if(q0 < 0.0f) q0 = 0.0f;
        if(q1 < 0.0f) q1 = 0.0f;
        if(q2 < 0.0f) q2 = 0.0f;
        if(q3 < 0.0f) q3 = 0.0f;
        q0 = sqrt(q0);
        q1 = sqrt(q1);
        q2 = sqrt(q2);
        q3 = sqrt(q3);
        if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
            q0 *= +1.0f;
            q1 *= SIGN(eerot[7] - eerot[5]);
            q2 *= SIGN(eerot[2] - eerot[6]);
            q3 *= SIGN(eerot[3] - eerot[1]);
        } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
            q0 *= SIGN(eerot[7] - eerot[5]);
            q1 *= +1.0f;
            q2 *= SIGN(eerot[3] + eerot[1]);
            q3 *= SIGN(eerot[2] + eerot[6]);
        } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
            q0 *= SIGN(eerot[2] - eerot[6]);
            q1 *= SIGN(eerot[3] + eerot[1]);
            q2 *= +1.0f;
            q3 *= SIGN(eerot[7] + eerot[5]);
        } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
            q0 *= SIGN(eerot[3] - eerot[1]);
            q1 *= SIGN(eerot[6] + eerot[2]);
            q2 *= SIGN(eerot[7] + eerot[5]);
            q3 *= +1.0f;
        } else {
            printf("Error while converting to quaternion! \n");
        }
        float r = NORM(q0, q1, q2, q3);
        q0 /= r;
        q1 /= r;
        q2 /= r;
        q3 /= r;
        pose.push_back(eetrans[0]);
        pose.push_back(eetrans[1]);
	pose.push_back(eetrans[2]);   	
	pose.push_back(q1); 
	pose.push_back(q2);   	
	pose.push_back(q3);   	
	pose.push_back(q0);   	  	

     }


bool Kinematics::isIKSuccess(std::vector<double> pose, std::vector<double>& joints){


     #if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif

#if IK_VERSION > 54
    //IKREAL_TYPE joints[num_of_joints];
    // for IKFast 56,61
    IkSolutionList<IKREAL_TYPE> solutions;
#else
            // for IKFast 54
    std::vector<IKSolution> vsolutions;
#endif
    std::vector<IKREAL_TYPE> vfree(num_free_parameters);
    eetrans[0] = pose[0];
    eetrans[1] = pose[1];
    eetrans[2] = pose[2];
    double qw = pose[6];
    double qx = pose[3];
    double qy = pose[4];
    double qz = pose[5];
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  
    eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         
    eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw; 
    eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz; 
    eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;   
    eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;    
    eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
    //for(std::size_t i = 0; i < vfree.size(); ++i)
           //vfree[i] = atof(argv[13+i]);
//TODO: the user have to define the number of free parameters for the manipulator if it has more than 6 joints. So currently more than 6 joints are not supported yet.

#if IK_VERSION > 54
            // for IKFast 56,61
            bool b1Success = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
	    
#else
            // for IKFast 54
            bool b2Success = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);


	
#endif
#if IK_VERSION > 54
	// for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            
            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
           
#if IK_VERSION > 54
            // for IKFast 56,61
if(!b1Success){
		return false;
		}
	else{
		//cout<<"Found ik solutions: "<< num_of_solutions<<endl; 
		const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(1);
                int this_sol_free_params = (int)sol.GetFree().size(); 
		std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
		sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
		joints=solvalues;
		return true;
	}
#else
if(!b2success){
		return false;
		}
	else{
		//cout<<"Found ik solutions: "<< num_of_solutions<<endl; 
		int this_sol_free_params = (int)vsolutions[i].GetFree().size();
		std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
		vsolutions[i].GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
		return true;
	}
#endif

}

const string Kinematics::getRobotName(){

	const char* hash=GetKinematicsHash();;
	
	string part=hash;
	part.erase(0,22);
	string name=part.substr(0,part.find(" "));
        return name;

}
};
