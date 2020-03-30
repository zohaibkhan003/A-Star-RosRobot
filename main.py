#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import matplotlib.pyplot as plt
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

counter = 0
waypoint_length = 0

class Turtlebot():
	def __init__(self):
    	rospy.init_node("turtlebot_move")
    	# some other code ...
    rospy.loginfo("Press CTRL + C to terminate")
    	rospy.on_shutdown(self.stop)

    	self.pose = Pose2D()
    	self.trajectory = list()
    	self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

    	self.vel = Twist()
    	self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

    	self.logging_counter = 0
    	self.rate = rospy.Rate(10)

    self.prev_vx = 0
    self.prev_vy = 0
    self.endvx = 0
    self.endvy = 0

    	try:
        	self.run()
    	except rospy.ROSInterruptException:
        	pass
    	finally:
        	self.visualization()

	def checkObstacle(self, nodes, obstacle):
    answer = False
    for x in obstacle:
    	if x == nodes:
        	answer = True
   	 return answer
    return answer

	def get_path_from_A_star(self):
    rospy.loginfo("Entered Get Path")

    #Choose the start point of the robot
    #input1 = input()
    #if input1 == 1:
    	start_point = [0, 0]
    #elif input1 == 2:
    #	start_point = [0, 0]
    #elif input1 == 3:
    #	start_point = [0, 0]
    #elif input1 == 4:
    #	start_point = [0, 0]
    #elif input1 == 5:
    #	start_point = [0, 0]
    #elif input1 == 6:
    #	start_point = [0, 0]

    	end_point = [7, 0.5]
    	obstacles = [[3, 0], [3, -1], [3, -2], [6, -1],[6, -0.5], [6,0], [6, 0.5], [6.5,-0.5], [6,1], [6,1.5], [6,2],[6.5, -0.5], [7, -0.5], [5.5,-1],[5.5, -0.5], [5.5, 0], [5.5, 0.5], [5.5, 1], [5,-1],[5, -0.5], [5, 0], [5, 0.5], [5, 1]]
    current_position = start_point
    gridx_max = 10
    gridy_max = 2
    gridx_min = 0
    gridy_min = -2

    open_list = []
    	closed_list = []
    	optimal_path = []
    G = []
    G_close = []
    H = []
    F = []
    parent_node = []
    parent_node_cl = []
    end = False

    while not end or not rospy.is_shutdown():
    	print ("Current position: ", current_position)
    	#Check neighboring points
    	for adjacent_position in [[0.5, 0], [0, 0.5], [0, -0.5], [-0.5, 0]]:
        	check_grid = [(adjacent_position[0] + current_position[0]),(adjacent_position[1] + current_position[1])]
        	# Check limits of grid
        	if (check_grid[0] > gridx_max or check_grid[0] < gridx_min or check_grid[1] > gridy_max or check_grid[1] < gridy_min):
            	continue

        	#Check if obstacle
        	flag = self.checkObstacle(check_grid, obstacles)
        	if(flag):
   	 	continue

   	 #Check if already a analyzed node
        	if check_grid in closed_list:
            	continue

        	#check if grid is in open list
        	if check_grid in open_list:
            	#parent_node.append(open_list.index(check_grid))
   	 	continue

        	#if check_grid is start point:
        	if check_grid == start_point:
            	continue
       	 
   	 #Check if endpoint
        	if check_grid == end_point:
            	end = True
            	break
       	 
        	open_list.append(check_grid)
        	if current_position != start_point:
   	 	parent_node.append(closed_list.index(current_position))
   	 else:
            	parent_node.append(0)
    	#print("Parent Node:", parent_node)
       	 
    	#print("Open List: ", open_list)

    	#Go through open list to find node with least weight
    	G = []
    	H = []
    	F = []
    	min_index = 0
    	duplicate_index = []
    	for j in range(len(open_list)):
        	if len(G_close) != 0 and (abs(start_point[0] - open_list[j][0]) > 0.5 and abs(start_point[1] - open_list[j][1]) > 0.5):
            	G.append(10 + G_close[parent_node[j]]) # 10 + G of parent node
   	 else:
   	 	G.append(10)
        	H.append(abs(end_point[0] - open_list[j][0]) + abs(end_point[1] - open_list[j][1]))
        	F.append(H[j] + G[j])
        	#parent_node[j] = prev_node
    	#print("G values: ", G)
    	#print("F values: ", F)

    	#Find the minimum
    	if len(F) != 0:
        	min_index = np.argmin(F)   
        	if  F.count(open_list[min_index]) > 1:
   	 	for k in range(len(open_list)):
                	if open_list[k] == open_list[min_index]:
   	         	duplicate_index.append(k)
   	 	for m in duplicate_index:
                	H_min.append(abs(end_point[0] - open_list[m][0]))
   	 	min_H_index = np.argim(H_min)
            	min_index = duplicate_index[min_H_index]
        	#if open_list[min_index] == [0,0.5]:
   		 
   	 #print("Minimum index: ", min_index)

    	if not end:
        	if min_index < len(open_list):
            	closed_list.append(open_list[min_index])
            	parent_node_cl.append(parent_node[min_index])
   	 	G_close.append(G[min_index])
            	current_position = open_list[min_index]
            	open_list.pop(min_index)
            	G.pop(min_index)
            	H.pop(min_index)
            	F.pop(min_index)
            	parent_node.pop(min_index)
            	#print("Closed Loop", closed_list)

            	prev_node  = current_position   	 
           	 
    	if end:
        	break
           	 
    	#self.rate.sleep()
    #Get optimal path from the closed path
    closed_list.append(end_point)

    #print("Closed List", closed_list)
    
    #for m in closed_list:
    #	if m == closed_list[0]:
    #    	optimal_path.append(m)
    #    	prevm = m
    #	if abs(m[0] - prevm[0]) <= 0.5 and abs(m[1] - prevm[1]) <= 0.5:
    #    	optimal_path.append(m)
    #    	prevm = m


    optimal_path.append(end_point)
    next_node = parent_node_cl[len(parent_node_cl)-1]

    while next_node != 0:
    	optimal_path.insert(0, closed_list[next_node])
    	print("Parent Node", parent_node_cl)
    	next_node = parent_node_cl[next_node]
    	print("Optimal Path", optimal_path)

    
    #change for proper grid 	CAN MAKE the grid bigger to avoid obstacles more, 0.55?
    for k in range(len(optimal_path)):
   	 	optimal_path[k][0] = optimal_path[k][0] * 0.53
   	 	optimal_path[k][1] = optimal_path[k][1] * 0.53
    	optimal_path[k][0] = optimal_path[k][0] - 0.27
   	 	optimal_path[k][1] = optimal_path[k][1] + 0.24

    #optimal_path = closed_list
    print("Right Grid Optimal Path", optimal_path)
    #optimal_path.append(end_point)
    	return optimal_path


	def run(self):
    	# get waypoints from A star algorithm
    global waypoint_length
    	WAYPOINTS = self.get_path_from_A_star()
    waypoint_length = len(WAYPOINTS)
    f = 0
    	for point in WAYPOINTS:
    	if f <= 0:
        	prevpoint = [0,0]
       	 
    	else:  	 
        	prevpoint = WAYPOINTS[f-1]
    	f = f+1
    	rospy.loginfo("In Run")
   	 
        	self.move_to_point(point, prevpoint)
    self.hit_ball()
    	self.stop()
    	rospy.loginfo("Action done.")

	def move_to_point(self, point, prevpoint):
    rospy.loginfo("In Move to point")
    xd = point[0]
    yd = point[1]    
    rospy.loginfo("Set Point: %s", xd)
    rospy.loginfo("Set Point Y: %s", yd)
    max_vel = 0.4
    max_acc = 0.2
    vel = Twist()
    global counter
    global waypoint_length

    kv = 0.7
    kw = 0.6
    M = 0.01
    
    #5th order polynomial
    T = 1.5
    x0 = point[0]
    xT = point[1]
    x0n = prevpoint[0]
    
    startvx = 0
    
    go = 0.23 #.17

    if prevpoint[0] > point[0]:
    	self.endvx = -1 * go
    elif prevpoint[0] == point[0]:
    	self.endvx = 0
    else:
    	self.endvx = 1 * go
    if prevpoint[1] > point[1]:
    	self.endvy = -1 * go
    elif prevpoint[1] == point[1]:
    	self.endvy = 0
    else:
    	self.endvy = 1 * go
    
    mat = np.array([[0, 0, 0, 0, 0, 1], [T**5, T**4, T**3, T**2, T, 1],[0, 0, 0, 0, 1, 0],[5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],[ 0, 0, 0, 2, 0, 0],[20*T**3, 12*T**2, 6*T, 2, 0, 0]])
    mat = np.linalg.inv(mat)
    print("mat", mat)
    counter = counter + 1
    print("Counter", counter)
    print("Previous X vel", self.prev_vx)
    print("Previous Y vel", self.prev_vy)
    print("End X vel", self.endvx)
    print("End Y vel", self.endvy)

    if counter  == 1:
    	ax = np.matmul(mat, np.array([[prevpoint[0]],[point[0]],[0],[self.endvx],[0],[0]]))
    	ay = np.matmul(mat, np.array([[prevpoint[1]],[point[1]],[0],[self.endvy],[0],[0]]))
    	self.prev_vx = self.endvx
    	self.prev_vy = self.endvy
    
    elif counter == waypoint_length:  #GET LENGTH of WAYPOINTS
    	ax = np.matmul(mat, np.array([[prevpoint[0]],[point[0]],[self.prev_vx],[0],[0],[0]]))
    	ay = np.matmul(mat, np.array([[prevpoint[1]],[point[1]],[self.prev_vy],[0],[0],[0]]))
   	 
    else:
        	ax = np.matmul(mat, np.array([[prevpoint[0]],[point[0]],[self.prev_vx],[self.endvx],[0],[0]]))
    	ay = np.matmul(mat, np.array([[prevpoint[1]],[point[1]],[self.prev_vy],[self.endvy],[0],[0]]))
    	self.prev_vx = self.endvx
    	self.prev_vy = self.endvy
    
    ax_T = ax.transpose()
    	ax = np.fliplr(ax_T)
    	ay_T = ay.transpose()
    	ay = np.fliplr(ay_T)

    t = np.r_[0: T: 0.1]    
    rospy.loginfo("****T is: %s", t)
    print ("AX", ax)
    print ("AY", ay)

    for i in t:
    	#t = i * 0.1
    	print ("i", i)
    	x = ax[0,0] + ax[0,1]*i + ax[0,2]*pow(i,2) + ax[0,3]*pow(i,3) + ax[0,4]*pow(i,4) + ax[0,5]*pow(i,5)
    	y = ay[0,0] + ay[0,1]*i + ay[0,2]*pow(i,2) + ay[0,3]*pow(i,3) + ay[0,4]*pow(i,4) + ay[0,5]*pow(i,5)
    	vx = ax[0,1] + 2*ax[0,2]*i + 3*ax[0,3]*pow(i,2) + 4*ax[0,4]*pow(i,3) + 5*ax[0,5]*pow(i,4)
    	vx8 = ax[0,1] + 2*ax[0,2]*0.8 + 3*ax[0,3]*pow(0.8,2) + 4*ax[0,4]*pow(0.8,3) + 5*ax[0,5]*pow(0.8,4)
    	vy = ay[0,1] + 2*ay[0,2]*i + 3*ay[0,3]*pow(i,2) + 4*ay[0,4]*pow(i,3) + 5*ay[0,5]*pow(i,4)
    	vy8 = ay[0,1] + 2*ay[0,2]*0.8 + 3*ay[0,3]*pow(0.8,2) + 4*ay[0,4]*pow(0.8,3) + 5*ay[0,5]*pow(0.8,4)
    	accx = 2*ax[0,2] + 6*ax[0,3]*i + 12*ax[0,4]*pow(i,2) + 20*ax[0,5]*pow(i,3)
    	accy = 2*ay[0,2] + 6*ay[0,3]*i + 12*ay[0,4]*pow(i,2) + 20*ay[0,5]*pow(i,3)
    	v = sqrt(pow(vx,2)+pow(vy,2))
    	acc = sqrt(pow(accx,2)+pow(accy,2))
    	print ("Vel x", vx)
    	print ("Vel y", vy)

    	#if vx > -9.0e-04 and vx < 9.0e-04:
     	#   vx = 0.0
    	#if vy > -9.0e-04 and vy < 9.0e-04:
    	#	vy = 0.0
    	print ("Vel x2", vx)
    	print ("Vel y2", vy)
    	theta_d = atan2(vy, vx)
    	if theta_d > 3.0:
        	theta_d = 0

    	#if i < 1.5: #0.8
     	#   theta_d = atan2(vy8, vx8)

    	rospy.loginfo("Goal theta: %s", theta_d)
    	etheta = theta_d - self.pose.theta
    	if etheta > pi:
        	etheta = etheta - 2*pi
    	elif etheta < -pi:
        	etheta = etheta + 2*pi

    	P = kw * (etheta) + M  #added etheta
    	D = kv * (etheta - (theta_d - self.pose.theta))
    	PD = P + D
    	if(v > max_vel):
        	rospy.loginfo("Velocity too High")
        	#v = max_vel
    	if(acc > max_acc):
   	 rospy.loginfo("Acc too High")
   	 
    	lastPD = PD
    	print("Linear Vel", v)
    	print("Angular Vel", PD)
    	vel.linear.x = v
    	vel.angular.z = PD
    	self.vel_pub.publish(vel)
        	self.rate.sleep()

	def hit_ball(self):
    vel = Twist()

    #robot face forward
   	 while self.pose.theta > 0.1 or self.pose.theta < -0.1:
    	if self.pose.theta > 0:
            	vel.angular.z = -0.5
    	else:
   	 vel.angular.z = 0.5
    	self.vel_pub.publish(vel)
    	self.rate.sleep()
    

	def visualization(self):
    	# plot trajectory
    	data = np.array(self.trajectory)
    	#np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
    	plt.plot(data[:,0],data[:,1])
    	plt.show()


	def stop(self):
    	# send zero velocity to robot
    	self.vel.linear.x = 0
    	self.vel.angular.z = 0
    	self.vel_pub.publish(self.vel)
    	rospy.sleep(1)


	def odom_callback(self, msg):
    	# Get (x, y, theta) specification from odometry topic
    	quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                	msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

    	self.pose.theta = yaw
    	self.pose.x = msg.pose.pose.position.x
    	self.pose.y = msg.pose.pose.position.y

    	# Logging once every 100 times
    	self.logging_counter += 1
    	if self.logging_counter == 100:
        	self.logging_counter = 0
        	self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
        	#rospy.loginfo("odom: x=" + str(self.pose.x) +\
        	#	";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
	try:
    	whatever = Turtlebot()
	except rospy.ROSInterruptException:
    	rospy.loginfo("Action terminated.")
