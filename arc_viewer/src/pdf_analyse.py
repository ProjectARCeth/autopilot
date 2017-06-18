#!/usr/bin/env python
'''
Info List:
0 - index_list
1 - index_path
2 - index_steering
3 - index_velocity
4 - distance_start
5 - distance_end
6 - tracking_error
7 - velocity
8 - should_velocity
9 - should_safe_velocity
10 - velocity_bound_physical
11 - velocity_bound_teach
12 - velocity_teach
13 - steering_angle
14 - should_steering_angle
15 - should_safe_steering_angle
16 - braking_distance
17 - radius
'''
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import os
import rospy
import sys

from ackermann_msgs.msg import AckermannDrive
from arc_msgs.msg import State
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Float32MultiArray

#Path to latex.
file_path = str(sys.argv[1])
#General constants.
v_freedom = 0.0
info_list_indexes = 18 # 18 information (0-17).
#Init ros.
rospy.init_node('analyse_pdf')
#General constants.
v_freedom = rospy.get_param("/control/V_FREEDOM")
k1_s = rospy.get_param("/control/K1_LAD_S")
k2_s = rospy.get_param("/control/K2_LAD_S")
k1_v = rospy.get_param("/control/K1_LAD_V")
k2_v = rospy.get_param("/control/K2_LAD_V")
#Topic names.
navigation_info_topic = rospy.get_param("/topic/NAVIGATION_INFO")
ready_for_driving_topic = rospy.get_param("/topic/VCU_LAUNCHING_COMMAND")
repeat_path_topic = rospy.get_param("/topic/PATH")
state_topic = rospy.get_param("/topic/STATE")
steering_angle_topic = rospy.get_param("/topic/STATE_STEERING_ANGLE")
stellgroessen_topic = rospy.get_param("/topic/STELLGROESSEN_SAFE")
teach_path_topic = rospy.get_param("/topic/TEACH_PATH")
tracking_error_topic = rospy.get_param("/topic/TRACKING_ERROR")

class Info:
	def __init__(self):
		# Init lists.
		self.info_list = np.zeros((1,info_list_indexes))
		self.ready = False
		self.repeat_path = np.zeros((1,2))
		self.teach_path = np.zeros((1,2))
		#Init Subscriber.
		rospy.Subscriber(navigation_info_topic, Float32MultiArray, self.navigationInfoCallback)
		rospy.Subscriber(ready_for_driving_topic, Bool, self.readyForDrivingCallback)
		rospy.Subscriber(repeat_path_topic, Path, self.repeatPathCallback)
		rospy.Subscriber(state_topic, State, self.stateCallback)
		rospy.Subscriber(steering_angle_topic, Float64, self.steeringAngleCallback)
		rospy.Subscriber(stellgroessen_topic, AckermannDrive, self.stellgroessenCallback)
		rospy.Subscriber(teach_path_topic, Path, self.teachPathCallback)
		rospy.Subscriber(tracking_error_topic, Float64, self.trackingErrorCallback)
	
	def createTxtEntryAtIndex(self, index):
		string = ""
		for column in range(0,info_list_indexes):
			string += str(round(self.info_list[index, column], 3))
			string += "\t"
		return string

	def getInfoList(self):
		return self.info_list

	def getLastInfoLine(self):
		last_index = self.info_list.size/info_list_indexes - 1
		last_line = self.info_list[last_index,:].tolist()
		return last_line

	def getListAtIndex(self, index):
		array = self.info_list[:,index]
		return array

	def getRepeatPath(self):
		return self.repeat_path.tolist()

	def getTeachPath(self):
		return self.teach_path.tolist()

	def navigationInfoCallback(self, msg):
		if(self.ready):
			line = self.getLastInfoLine()
			line[4] = msg.data[0] # Distance start.
			line[5] = msg.data[1] # Distance end.
			line[2] = msg.data[2] # Index steering.
			line[14] = msg.data[3] # Should steering angle.
			line[3] = msg.data[4] # Index velocity.
			line[17] = msg.data[5] # Radius.
			line[10] = msg.data[6]*3.6 # Velocity bound physical.
			line[16] = msg.data[7] # Braking distance.
			line[11] = msg.data[8]*3.6 # Velocity bound teach.
			line[12] = (msg.data[8] - v_freedom)*3.6 # Velocity teach.
			line[8] = msg.data[9] # Should velocity.
			self.setNewInfoLine(line)

	def readyForDrivingCallback(self, msg):
		if(msg.data): self.ready = True

	def repeatPathCallback(self, msg):
		self.repeat_path = np.zeros((1,2))
		for element in msg.poses:
			path_element = np.array([-element.pose.position.y, element.pose.position.x])
			self.repeat_path = np.vstack([self.repeat_path, path_element])

	def setNewInfoLine(self, array):
		array[0] += 1
		np_array = np.array(array)
		self.info_list = np.vstack([self.info_list, np_array])

	def stateCallback(self, msg):
		if(self.ready):
			line = self.getLastInfoLine()
			line[1] = msg.current_arrayposition # Index path.
			line[7] = msg.pose_diff*3.6 # Velocity.
			self.setNewInfoLine(line)

	def steeringAngleCallback(self, msg):
		if(self.ready):
			line = self.getLastInfoLine()
			line[13] = msg.data # Steering angle.
			self.setNewInfoLine(line)

	def stellgroessenCallback(self, msg):
		if(self.ready):
			line = self.getLastInfoLine()
			line[15] = msg.steering_angle # Safe steering angle.
			line[9] = msg.speed # Safe velocity.
			self.setNewInfoLine(line)
		
	def teachPathCallback(self, msg):
		self.teach_path = np.zeros((1,2))
		for element in msg.poses:
			path_element = np.array([-element.pose.position.y, element.pose.position.x])
			self.teach_path = np.vstack([self.teach_path, path_element])

	def trackingErrorCallback(self, msg):
		if(self.ready):
			line = self.getLastInfoLine()
			line[6] = msg.data # Tracking error.
			self.setNewInfoLine(line) 

def getIndexArray(array):
	index_array = []
	index = 0
	for element in array:
		index_array.append(index)
		index += 1
	return index_array

def getTwoArrays(array):
	x = []
	y = []
	for i in range(0, len(array)):
		x.append(array[i][0])
		y.append(array[i][1])
	return x,y
	
if __name__ == '__main__':
	#Init info class.
	information = Info()
	#Init subscribing loop.
	rospy.spin()
	#Create plots.
	fig = plt.figure(figsize=(10, 7))
	gs = gridspec.GridSpec(4, 4)
	gs.update(hspace=0.4)

	ax0 = plt.subplot(gs[0, :4])
	plt.title("Path Analysis")
	tracking_error = information.getListAtIndex(6)
	index_array = getIndexArray(tracking_error)
	plt.plot(tracking_error)
	plt.ylabel('tracking_error[m]')

	ax1 = plt.subplot(gs[1, :4])
	velocity = information.getListAtIndex(7)
	index_base = getIndexArray(velocity)
	plt.plot(index_base, velocity, 'b', label="repeat")
	velocity_teach = information.getListAtIndex(12)
	index_target = getIndexArray(velocity_teach)
	plt.plot(index_target, velocity_teach, 'g', label="teach")
	plt.ylabel('velocity[km/h]')
	# plt.legend(bbox_to_anchor=(0.,1.02,1.,.102),loc=2,ncol=2)

	ax2 = plt.subplot(gs[2:4,:2])
	repeat_path = information.getRepeatPath()
	teach_path = information.getTeachPath()
	teach_x, teach_y = getTwoArrays(teach_path)
	repeat_x, repeat_y = getTwoArrays(repeat_path)
	plt.plot(teach_x, teach_y, 'go', label="teach")
	plt.plot(repeat_x, repeat_y, 'bo', label="repeat")
	plt.ylabel('Teach and Repeat path')
	
	ax3 = plt.subplot(gs[2:3,2:4])
	plt.axis('off')
	frame = plt.gca()
	frame.axes.get_xaxis().set_ticks([])
	frame.axes.get_yaxis().set_ticks([])
	distance_start = information.getListAtIndex(4)
	path_vals =[['Distance[m]', round(max(distance_start),3),'',''],
	 			['K1_S: '+str(k1_s),'K2_S: '+str(k2_s),'K1_V: '+str(k1_v), 'K2_V: '+str(k2_v)]]
	path_table = plt.table(cellText=path_vals,
	                  	   colWidths = [0.1]*4,
	                       loc='center left')
	path_table.set_fontsize(14)
	path_table.scale(2.2,3)

	ax4= plt.subplot(gs[3:4,2:4])
	plt.axis('off')
	frame = plt.gca()
	frame.axes.get_xaxis().set_ticks([])
	frame.axes.get_yaxis().set_ticks([])
	mean_labels=['','Mean','Variance','Median']
	mean_vals=[['Track Error',round(np.mean(tracking_error),3),round(np.var(tracking_error),3),round(np.median(tracking_error),3)],
				['Velocity',round(np.mean(velocity),3),round(np.var(velocity),3),round(np.median(velocity),3)]]
	mean_table = plt.table(cellText=mean_vals,
	                  	   colWidths = [0.1]*4,
	                  	   colLabels=mean_labels,
	                  	   loc='center left')
	mean_table.set_fontsize(14)
	mean_table.scale(2.2,3)

	plt.savefig(file_path+"_infos.png")
	plt.close()

	#Create bigger paths.
	fig2 = plt.figure(figsize=(20, 14))
	plt.plot(teach_x, teach_y, 'go', label="teach")
	plt.plot(repeat_x, repeat_y, 'bo', label="repeat")
	plt.savefig(file_path+"_path.png")
	plt.close()
	
	#Create txt file table.
	file = open(file_path+"_infos.txt", "w")
	file.write("InLi\tInPa\tInSt\tInVe\tDiSt\tDiEn\tTrEr\tVel\tVeSh\tVeSa\tVeBP\tVeBT\tVeTe\tStAn\tStSh\tStSa\tBrDi\tRad\n")
	for index in range(0, len(tracking_error)):
		string = information.createTxtEntryAtIndex(index)
		file.write(string)
		file.write("\n")
	file.close()


