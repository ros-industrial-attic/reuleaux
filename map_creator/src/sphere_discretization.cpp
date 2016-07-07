#include <map_creator/sphere_discretization.h>


namespace sphere_discretization{

//SphereDiscretization::SphereDiscretization(){}

OcTree* SphereDiscretization::generateSphereTree(point3d origin, float radius, float resolution){
    OcTree* tree=new OcTree(resolution);
	point3d point_on_surface=origin;
	
	point_on_surface.x() +=radius;
	for (int i=0; i<360; i++){
		for (int j=0;j<360; j++){
			
			if(!tree ->insertRay(origin, origin+point_on_surface)){
				cout<<"Error while inserting ray from "<<origin<<" to "<<point_on_surface<<endl;
			
			}
			point_on_surface.rotate_IP(0,0,DEG2RAD(1.));
		}
		point_on_surface.rotate_IP(0,DEG2RAD(1.0),0);
	}
return tree;
}

OcTree* SphereDiscretization::generateSphereTree2(point3d origin, float radius, float resolution){
    OcTree* tree=new OcTree(resolution);
	point3d point_on_surface=origin;
	point_on_surface.x() +=radius;
	unsigned sphere_beams=500;
        double angle =2.0*M_PI/double(sphere_beams);
        Pointcloud p;
        for(unsigned i=0;i<sphere_beams; i++){
	  for(unsigned j=0;j<sphere_beams; j++){
		p.push_back(origin+point_on_surface);
		point_on_surface.rotate_IP(0,0,angle);
		}
	point_on_surface.rotate_IP(0,angle,0);
	}
    tree->insertPointCloud(p,origin);
return tree;
}

OcTree* SphereDiscretization::generateBoxTree(point3d origin, float diameter, float resolution){
    OcTree* tree=new OcTree(resolution);
	Pointcloud p;
	for (float x=origin.x()-diameter*1.5; x <=origin.x()+ diameter*1.5; x+=resolution*2){

    		for (float y=origin.y()-diameter*1.5; y <= origin.y()+ diameter*1.5; y+= resolution*2){
      			for (float z=origin.z()-diameter*1.5; z <= origin.z()+diameter*1.5; z+= resolution*2){
				  //tree ->insertRay(origin, point3d(x,y,z));
				  point3d point;
				  point.x()=x;
				  point.y()=y;
				  point.z()=z;
				  tree ->updateNode(point,true);
				}
			}
		}
	
return tree;
};

Pointcloud SphereDiscretization::make_sphere_points(point3d origin, double r){
    Pointcloud spherePoints;
	for (double phi = 0.; phi < 2*M_PI; phi += M_PI/7.) // Azimuth [0, 2PI]
    {
        for (double theta = 0.; theta < M_PI; theta += M_PI/7.) // Elevation [0, PI]
        {
            point3d point;
            point.x() = r * cos(phi) * sin(theta) + origin.x();
            point.y() = r * sin(phi) * sin(theta) + origin.y();
            point.z() = r * cos(theta) + origin.z();
            spherePoints.push_back(point);        
        }
    }
    return spherePoints;
}


vector<geometry_msgs::Pose> SphereDiscretization::make_sphere_poses(point3d origin, double r){

   vector<geometry_msgs::Pose> pose_Col;
    geometry_msgs::Pose pose;
//TODO Most of the robots have a roll joint as their final joint which can move 0 to 2pi. So if a pose is reachable, then the discretization of roll poses are also reachable. It will increase the data, so we have to decide if we have to use it. The robot whose final roll joint cannot move 0 to 2pi, we have to use it. 
//for (double rot = 0.; rot < 2*M_PI; rot += M_PI/6){
    
	for (double phi = 0.; phi < 2*M_PI; phi += M_PI/5.) // Azimuth [0, 2PI]
    {
        for (double theta = 0.; theta < M_PI; theta += M_PI/5.) // Elevation [0, PI]
        {
            
            pose.position.x = r * cos(phi) * sin(theta) + origin.x();
            pose.position.y = r * sin(phi) * sin(theta) + origin.y();
            pose.position.z = r * cos(theta) + origin.z();
	    tf2::Quaternion quat;
            //tf2::Quaternion quat2;
	    //quat2.setRPY(M_PI/6,0,0);
            quat.setRPY(0,((M_PI/2)+theta), phi);
	    //quat=quat*quat2; 
	    quat.normalize();
	    pose.orientation.x=quat.x();
	    pose.orientation.y=quat.y();
	    pose.orientation.z=quat.z();
	    pose.orientation.w=quat.w();
            pose_Col.push_back(pose);
        	}
            }
      //}
    return pose_Col;
}

double SphereDiscretization::irand(int min, int max){
    return ((double)rand() / ((double)RAND_MAX + 1.0)) * (max - min) + min;

}

Pointcloud SphereDiscretization::make_sphere_rand(point3d origin, double r, int sample){
    Pointcloud spherePoints;
	double theta=0, phi=0;
        for (int i=0;i<sample;i++) 
        {
            point3d point;
            theta=2*M_PI*irand(0,1);
	    phi=acos(2*irand(0,1)-1.0);
            point.x() = float(r * cos(theta) * sin(phi) + origin.x());
            point.y() = float(r * sin(theta) * sin(phi) + origin.y());
            point.z() = float(r * cos(phi) + origin.z());
            spherePoints.push_back(point);        
            //cout<<theta<<endl;
    }
    return spherePoints;
}

Pointcloud SphereDiscretization::make_sphere_Archimedes(point3d origin, double r, int sample){
    Pointcloud spherePoints;
	double theta=0, phi=0;
        for (int i=0;i<sample/2;i++) 
        {
            point3d point;
	    point3d point2;
            theta=2*M_PI*irand(-1,1);
	    point.z() = r*irand(0,1) + origin.z();
            point.x() = r*sqrt(1-point.z()*point.z())*cos(theta)+origin.x();
            point.y() = r*sqrt(1-point.z()*point.z())*sin(theta)+origin.y();
            point2.z()=-point.z();
	    point2.y()=-point.y();
	    point2.x()=-point.x();
            spherePoints.push_back(point);  
	    spherePoints.push_back(point2);              
            cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
    }
    return spherePoints;
}

Pointcloud SphereDiscretization::make_sphere_fibonacci_grid(point3d origin, double r, int sample){

double ng;
	double r_phi=( 1.0 + sqrt ( 5.0 ) ) / 2.0;
        Pointcloud spherePoints;
	ng=double(sample);
	double theta=0, phi=0;
        for (int i=0;i<sample;i++) 
        {
	    double i_r8=(double)(-ng+1+2*i);
            point3d point;
	    point3d point2;
            theta=2*r_phi*i_r8/M_PI;
	    double sphi = i_r8 / ng;
	    double cphi = sqrt ( ( ng + i_r8 ) * ( ng - i_r8 ) ) / ng;
	    
            point.x() = r*cphi*sin(theta)+origin.x();
            point.y() = r*cphi*cos(theta)+origin.x();
            point.z() = r*cphi;
	    point2.z()=-point.z();
	    point2.y()=-point.y();
	    point2.x()=-point.x();
            spherePoints.push_back(point); 
            spherePoints.push_back(point2);        
            //cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
	    //cout<< i_r8<<endl;

    }
    return spherePoints;
}

double SphereDiscretization::r8_modp(double x, double y){

//Returns non-negetive remainder of R8 division.
   double value;
   if(y=0.0)
   {
	cerr<<"R8-MODP error: called with the y value of "<<y<<"\n";
        exit(1);
    }
   value=x-((double)((int)(x/y)))*y;
   if(value<0.0)
   {
	value=value+fabs(y);
   }
   return value;
}
    
Pointcloud SphereDiscretization::make_sphere_spiral_points(point3d origin, double r, int sample){
    Pointcloud spherePoints;
        
	double theta=0, phi=0;
	double sinphi=0, cosphi=0;
        for (int i=0;i<sample;i++) 
        {
            point3d point(0,0,0);
	    cosphi=((double)(sample-i-1)*(-1.0)+(double)(i)*(1.0))/(double)(sample-1);
	    sinphi=sqrt(1.0-cosphi*cosphi);
	    if(i==0 || i==sample-1){
		   theta=0.0;
		}
	    else{
		
            	   theta=theta+3.6/(sinphi*sqrt((double)sample));
		   theta=r8_modp(theta, 2.0*M_PI);
		}

	    point.x() = r* sinphi* cos(theta) + origin.x();
            point.y() = r* sinphi* sin(theta) + origin.y();
            point.z() = r* cosphi +origin.y();
            
            spherePoints.push_back(point);  
	         
            //cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
    }
    return spherePoints;
}


Pointcloud SphereDiscretization::make_long_lat_grid(point3d origin, double r, int sample, int lat_num, int lon_num){
    Pointcloud spherePoints;
        
	double theta=0, phi=0;
	point3d point;
        point.x() = r* sin(phi)* cos(theta) + origin.x();
        point.y() = r* sin(phi)* sin(theta) + origin.y();
        point.z() = r* cos(phi) +origin.y();
        for (int lat=1;lat<lat_num;lat++) 
        {
            phi=(double)(lat)*M_PI/(double)(lat_num+1);
	    for (int lon=0;lon<lon_num;lon++){
	    {
		theta=(double)(lon)*2.0*M_PI/(double)(lon_num);
	        point.x() = r* sin(phi)* cos(theta) + origin.x();
                point.y() = r* sin(phi)* sin(theta) + origin.y();
                point.z() = r* cos(phi) +origin.y();
                sample=sample+1;
                spherePoints.push_back(point);  
	      }
	    }
	point.x() = r* sin(phi)* cos(theta) + origin.x();
        point.y() = r* sin(phi)* sin(theta) + origin.y();
        point.z() = r* cos(phi) +origin.y();
        sample=sample+1;       
            cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
    }
    return spherePoints;
}


void SphereDiscretization::convertPointToVector(const point3d point, std::vector<double> & data){
	
    data.push_back(double(point.x()));
    data.push_back(double(point.y()));
    data.push_back(double(point.z()));
	
	}


void SphereDiscretization::convertVectorToPoint(const std::vector<double> data, point3d & point){
	
    point.x()=data[0];
    point.y()=data[1];
    point.z()=data[2];
    }


void SphereDiscretization::convertPoseToVector(const geometry_msgs::Pose pose, std::vector<double> & data){
	
    data.push_back(double(pose.position.x));
    data.push_back(double(pose.position.y));
    data.push_back(double(pose.position.z));
    data.push_back(double(pose.orientation.x));
    data.push_back(double(pose.orientation.y));
    data.push_back(double(pose.orientation.z));
    data.push_back(double(pose.orientation.w));
	
	}

void SphereDiscretization::convertVectorToPose(const std::vector<double> data, geometry_msgs::Pose & pose){
	
    pose.position.x=data[0];
    pose.position.y=data[1];
    pose.position.z=data[2];
    pose.orientation.x=data[3];
    pose.orientation.y=data[4];
    pose.orientation.z=data[5];
    pose.orientation.w=data[6];
    }


geometry_msgs::Pose SphereDiscretization::findOptimalPose(const vector<geometry_msgs::Pose> poses, point3d origin){

    geometry_msgs::Pose optiPose;
    double mydo[] = {0,0,0,0,0,0};
    std::vector<double> mean (mydo, mydo + sizeof(mydo) / sizeof(double) ); 
    for(int i=0;i<poses.size();i++)
    {
	
        mean[0] +=poses[i].position.x;
	mean[1] +=poses[i].position.y;
	mean[2] +=poses[i].position.z;
	mean[3] +=poses[i].orientation.x;
	mean[4] +=poses[i].orientation.y;
	mean[5] +=poses[i].orientation.z;
	mean[6] +=poses[i].orientation.w;
        }
    tf2::Quaternion quat(mean[3]/poses.size(),mean[4]/poses.size(),mean[5]/poses.size(),mean[6]/poses.size());
    quat.normalize();
    optiPose.position.x=origin.x();
    optiPose.position.y=origin.y();	
    optiPose.position.z=origin.z();
    optiPose.orientation.x=quat.x();
    optiPose.orientation.y=quat.y();
    optiPose.orientation.z=quat.z();
    optiPose.orientation.w=quat.w();

    return optiPose;
}

void SphereDiscretization::createConeCloud(const geometry_msgs::Pose pose, const double opening_angle, const double scale, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    float angle=M_PI/opening_angle;
    static const double delta_theta = M_PI / 32.0;
    double theta=0;
    point3d origin=point3d(0,0,0);
    for(float j=0;j<=angle/2;j+=0.01){
        for(float k=0;k<=scale;k+=0.001){
  	    pcl::PointXYZ point;
    	    point.x = origin.x()+k;
            point.y = origin.y()+k *cos(theta+delta_theta)*j;
            point.z = origin.z()+k *sin(theta+delta_theta)*j;
            theta +=delta_theta;
            new_cloud->push_back(point);	
		}
	    }
    
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Quaternionf q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    q.normalize();
    transform.translation() << pose.position.x,pose.position.y,pose.position.z;
    
    transform.rotate(q);
    transform.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*new_cloud, *cloud, transform);
   
}

void SphereDiscretization::poseToPoint(const geometry_msgs::Pose pose, point3d & point){
    point.x()=pose.position.x;
    point.y()=pose.position.y;
    point.z()=pose.position.z;
}
bool SphereDiscretization::isPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, point3d point){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setComputeAreaVolume(true);
    chull.reconstruct(*cloud_hull);
    pcl::PointXYZ query_point;
    query_point.x=float(point.x());
    query_point.y=float(point.y());
    query_point.z=float(point.z());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
    
        for (size_t j = 0; j < cloud->points.size (); ++j)
  	{
	    cloud_new->push_back(cloud->points[j]);
	}
	cloud_new->push_back(query_point);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_check (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull_check;
        chull_check.setInputCloud (cloud_new);
        chull_check.setComputeAreaVolume(true);
        chull_check.reconstruct(*cloud_hull_check);
	if (chull_check.getTotalArea()-chull.getTotalArea()<=0.05){
		return true;
		}
	else
	{
		return false;
		}
	}
    
     


};
