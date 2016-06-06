#!/usr/bin/env python
import rospy
import h5py
import numpy as np
from reuleaux.msg import PointArray
from geometry_msgs.msg import Point32

#TODO database path a parameter
database_file="/home/abhijit/Desktop/reachability.12a06408e3f80af9e5cd98f6fe50e0ba.pp"

def get_data():

	f=h5py.File(database_file,"r")
	
	rstat=f["reachabilitystats"]
	
	rospy.init_node('points_from_hdf5')
	rate = rospy.Rate(10) 
	
	pub=rospy.Publisher('points', PointArray, queue_size=10,latch=True)
	
	while not rospy.is_shutdown():
		po=PointArray()
		#po.point_len=len(rstat)
#To visualize the whole map, put po.point_len instead of the number
#TODO make count a parameter
		for i in range(0,1000):
			
			p=Point32()
			
			p.x=rstat[i][4]
			p.y=rstat[i][5]
			p.z=rstat[i][6]
			rospy.loginfo('sending data #[%i] [%f] [%f] [%f]',i,p.x, p.y, p.z  ) 
			po.points.append(p)
		#rospy.loginfo('Ye lo data # [%i]',po.point_len)
		pub.publish(po)
        	
		rospy.sleep(2)
	

if __name__ == '__main__':
    
    get_data()
     
