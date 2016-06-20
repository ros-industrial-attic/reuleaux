#!/usr/bin/env python
import rospy
import h5py
import numpy as np
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

#TODO database path a parameter
database_file="/home/abhijit/Desktop/reachability.12a06408e3f80af9e5cd98f6fe50e0ba.pp"

def get_data():

	f=h5py.File(database_file,"r")
	
	rstat=f["reachabilitystats"]
	
	rospy.init_node('points_from_hdf5')
	rate = rospy.Rate(10) 
	
	pub=rospy.Publisher('orientations', PoseArray, queue_size=10,latch=True)
	
	while not rospy.is_shutdown():
		po=PoseArray()
		po.header.stamp=rospy.Time.now()
		po.header.frame_id="/my_frame"
#To visualize the whole map, put rstat.len() instead of the number
#TODO make count a parameter
		for i in range(0,10000):
			
			p=Pose()
			p.orientation.w=rstat[i][0]
			p.orientation.x=rstat[i][1]
			p.orientation.y=rstat[i][2]
			p.orientation.z=rstat[i][3]
			p.position.x=rstat[i][4]
			p.position.y=rstat[i][5]
			p.position.z=rstat[i][6]
			rospy.loginfo("sending data #[%i]", i ) 
			po.poses.append(p)
		#rospy.loginfo('Ye lo data # [%i]',po.point_len)
		pub.publish(po)
        	
		rospy.sleep(2)
	

if __name__ == '__main__':
    
    get_data()
     
