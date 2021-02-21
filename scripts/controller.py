#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_srvs.srv import *

## Global variable that check if a target is reached
reached_ = None

## Global Publisher to set a new goal
pub_goal_ = None

## Global Publisher to stop the velocity of the robot
pub_stop_vel_ = None

def setGoal():
	"""
		Function used to publish a new goal.
		A variable "goal" is set with the new coordinates. The two
		coordinates (x,y) are read in the two parameters "des_pos_x" and
		"des_pos_y".
		When the variable goal is correctly set, it'll be published
		through the publisher pub_goal_.
	"""
	global pub_goal_
	
	goal = MoveBaseActionGoal()

	goal.goal.target_pose.header.frame_id = "map"
	goal.goal.target_pose.pose.orientation.w = 1
	goal.goal.target_pose.pose.position.x = rospy.get_param('des_pos_x')
	goal.goal.target_pose.pose.position.y = rospy.get_param('des_pos_y')

	pub_goal_.publish(goal)

def stopRobot():
	"""
		Function used to stop the velocity of the robot.
		Using the topic '/cmd_vel' the linear and angular velocity is
		published with value zero through the publisher pub_stop_vel_.
	"""
	global pub_stop_vel_
	
	stop_vel = Twist()
	stop_vel.linear.x = 0
	stop_vel.linear.y = 0
	stop_vel.angular.z = 0

	pub_stop_vel_.publish(stop_vel)

## Callback function of subscriber to chek if a terget is reached
def targetReachedCallback(result):
	"""
		Callback function used to notify the controller when a status of
		the robot is changed.
		The status checked is the one "SUCCEEDED", target reached.
		It is useful to understand when a new command can be sent.
		
		Args:
			result - it contains the status of the robot during move_base
					 algorithm, described by the /move_base/result topic
	"""
	global reached_
	
	status = result.status.status
	
	if status == result.status.SUCCEEDED:
		reached_ = True
		rospy.set_param("/bug0_reached", reached_)
		print "------------Target Reached!------------"

def control():
	"""
		Main function that manages the behaviours of the robot.
		In this function are set two publishers: pub_goal_ and
		pub_stop_vel, the first one is used to set a new goal and the 
		second one is used to stop the robot after reaching a target or
		when it is following the wall.
		Here are also set some Service as the userCommand, randomTarget,
		userTarget, wall_follower and the bug0.
		It is present also a Subscriber useful to understand the status
		of the robot, in particular if a target is reached.
	"""
	global pub_goal_, reached_, pub_stop_vel_
	
	# wait 10 second in order to wait that all service starts
	time.sleep(10)
	
	# the target is set as reached
	reached_ = True
	
	rospy.init_node('controller')

	# getting the deafult command (0)
	command = rospy.get_param("/command")

	# subscriber useful to understand if the robot reach the target point
	rospy.Subscriber("/move_base/result", MoveBaseActionResult, targetReachedCallback)

	# publisher used to set a new goal
	pub_goal_ = rospy.Publisher('/move_base/goal', MoveBaseActionGoal , queue_size=1)
	
	# publisher used to set the velocity of the robot
	pub_stop_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	# service used to read a command sent by the user
	srv_user_command = rospy.ServiceProxy('/userCommand_service', SetBool)
	srv_user_command(True)

	# service that provides a random target
	srv_random_target = rospy.ServiceProxy('/randomTarget_service', Empty)

	# service that allows the user to type a new target
	srv_user_target = rospy.ServiceProxy('/userTarget_service', Empty)

	# client that follow the wall
	srv_wall_follower = rospy.ServiceProxy('wall_follower_service', SetBool)
	srv_wall_follower(False)

	# client that use the algorithm bug0
	srv_client_bug0 = rospy.ServiceProxy('bug0_service', SetBool)
	
	# the algorithm is set as false by default, because the program 
	# starts with the move_base algorithm
	isBug0 = False
	srv_client_bug0(isBug0)

	# variable that avoid repeating several times the command wall, stop
	# and bug0. They are used because is useless repeat more than one
	# time the same command where it is waiting for a new one.
	alreadyDone_wall = False
	alreadyDone_stop = False
	alreadyDone_bug0 = False
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():

		# for each cycle it reads the command set one the param.
		command = rospy.get_param("/command")
		
		# if the bug0 is active, reads the reached value from the param
		if isBug0:
			reached_ = rospy.get_param("/bug0_reached")

		# if the command 1 (random target) is set and the target is reached
		if command == 1 and reached_:
			reached_ = False
			rospy.set_param("/bug0_reached", reached_)
			
			# reset value of the variables
			alreadyDone_wall = False
			alreadyDone_stop = False
			alreadyDone_bug0 = False
			
			# stop wall follower
			srv_wall_follower(False)
			# stop the robot in order to reach the new target
			stopRobot()
			
			# read the next random target.
			srv_random_target()
			print ("Random target position [" + str(rospy.get_param('des_pos_x')) + ", " + str(rospy.get_param('des_pos_y')) + "]\n")
			
			# if the bug0 algorithm isn't set, set the goal for move_base
			if not isBug0:
				setGoal()
			# else activate bug0
			elif isBug0:
				srv_client_bug0(isBug0)
			
			# allow the user to type a new command
			srv_user_command(True)

		# if the command 2 (user target) is set and the target is reached
		elif command == 2 and reached_:
			reached_ = False
			rospy.set_param("/bug0_reached", reached_)
			
			# reset value of the variables
			alreadyDone_wall = False
			alreadyDone_stop = False
			alreadyDone_bug0 = False

			# stop the user command in order to give the possibility
			# to insert a new target
			srv_user_command(False)
			# stop wall follower
			srv_wall_follower(False)
			# stop the robot in order to reach the new target
			stopRobot()
			
			# read the user target through the service
			srv_user_target()
			
			# if the bug0 algorithm isn't set, set the goal for move_base
			if not isBug0:
				setGoal()	
			# else activate bug0
			elif isBug0:
				srv_client_bug0(isBug0)

		# if the command 3 is set (wall follower) and the target is reached
		elif command == 3 and reached_:
			# if the command isn't already activated
			if not alreadyDone_wall:
				print "\nWall follower activated"
				
				# if bug0 is activated, disable it in ordet to do the wall follower
				if isBug0:
					srv_client_bug0(False)
				
				# reset value of the variables
				alreadyDone_stop = False
				alreadyDone_bug0 = False
				
				# set true the wall follower variable
				alreadyDone_wall = True
				
				# activate the wall follower service
				srv_wall_follower(True)
				
				# reset target position
				rospy.set_param("des_pos_x", 0)
				rospy.set_param("des_pos_y", 0)
				
				# allow the user to insert a new command
				srv_user_command(True)

		# if the command is 4 (stop robot) and the target is reached
		elif command == 4 and reached_:
			# if the command isn't already activated
			if not alreadyDone_stop:
				print "\nStopping the robot"
				
				# reset value of the variables
				alreadyDone_wall = False
				alreadyDone_bug0 = False
				
				# set true the stop_robot variable
				alreadyDone_stop = True
				
				# stopping wall follower
				srv_wall_follower(False)
				stopRobot() # stopping the robot
				# allow the user to insert a new command
				srv_user_command(True)

		# if the command 5 (bug0 algorithm) and the target is reached
		elif command == 5 and reached_: # change algorithm
			# if the command isn't already activated
			if not alreadyDone_bug0:
				
				# reset value of the variables
				alreadyDone_wall = False
				alreadyDone_stop = False
				
				# set true bug0 variable
				alreadyDone_bug0 = True
				
				# switching the algorithm (true: Bug0, false: Move_base)
				isBug0 = not isBug0
				srv_client_bug0(isBug0)
				
				# stopping the wall follower
				srv_wall_follower(False)
				stopRobot() # stopping the robot
				# allow the user to insert a new command
				srv_user_command(True)
				
				# print the actual algorithm
				if isBug0:
					print "\nSwtiching to Bug0 algorithm"
				else:
					print "\nSwitching to Move Base algorithm"

		rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
