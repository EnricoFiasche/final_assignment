#! /usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Twist
from std_srvs.srv import *

import math

## Publisher for the velocity
pub = None

## service go to point
srv_client_go_to_point_ = None

## service wall follower
srv_client_wall_follower_ = None

## angle set to 0
yaw_ = 0

## variable that contains the error allowed 
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees

## position of the robot
position_ = Point()

## desidered position
desired_position_ = Point()

## previous value of the coordinate x
prev_value_x = 0

## previous value of the coordinate y
prev_value_y = 0

## setting the coordinates of the desidered position
desired_position_.x = rospy.get_param('des_pos_x')

## setting the coordinates of the desidered position
desired_position_.y = rospy.get_param('des_pos_y')

## setting the coordinates of the desidered position
desired_position_.z = 0

## regions of the laser
regions_ = None

## States of the algorithm
state_desc_ = ['Go to point', 'wall following', 'target reached']

## default state, target reached
state_ = 2

## Varible useful to understand if the algorithm is activated
active_ = False

def bug0_switch(req):
	"""
		Function that enable and disable the gub0 algorithm
		
		Args:
			req - Request data, which could be True or False
	"""
	global active_

	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'Done!'
	return res
    
def clbk_odom(msg):
	"""
		CallBack function that reads the position of the robot using the
		topic "/odom"
		
		Args:
			msg - message that contains the position of the robot
				  described by the /odom topic
	"""
	global position_, yaw_

    # position
	position_ = msg.pose.pose.position

    # yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def clbk_laser(msg):
	"""
		CallBack function used to read the laser value that change
		the value of each region.
		
		Args:
			msg - message that contains the values read by the laser
			      described by the /scan topic.
	"""
	global regions_
	regions_ = {
		'right':  min(min(msg.ranges[0:143]), 10),
		'fright': min(min(msg.ranges[144:287]), 10),
		'front':  min(min(msg.ranges[288:431]), 10),
		'fleft':  min(min(msg.ranges[432:575]), 10),
		'left':   min(min(msg.ranges[576:719]), 10),
	}


def change_state(state):
	"""
		Function that allow the bug0 algorithm to change the state.
		
		Args:
			state - used to understand in which state is the robot.
					The state are three:
						0 - Go to point
						1 - Wall follower
						2 - Target reached
	"""
	global state_, state_desc_
	global srv_client_wall_follower_, srv_client_go_to_point_
	global active_, prev_value_x, prev_value_y

	state_ = state

	if state_ == 0:
		resp = srv_client_go_to_point_(True)
		resp = srv_client_wall_follower_(False)

	if state_ == 1:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(True)

	if state_ == 2:
		resp = srv_client_go_to_point_(False)
		resp = srv_client_wall_follower_(False)
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
        
		# if the robot is activated
		if active_:
			print "------------Target Reached!------------"
			
			# change the value of the param reached
			rospy.set_param("/bug0_reached", True)

			# updating the previous target point
			prev_value_x = rospy.get_param('des_pos_x')
			prev_value_y = rospy.get_param('des_pos_y')

def normalize_angle(angle):
	"""
		Function used to normalize the angle passed by parameter
		
		Arg:
			angle - Angle that has to be normalized
	"""
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle

def main():
	"""
		Main function that controls the robot during the motion using bug0.
		It controls the robot through three state
		0 - go to point
		1 - wall follower
		2 - target reached
		During the motion the program checks the distance from the actual
		position and the target, if it is less than a given threshold, the
		state of the robot is switched to target reached.
		A timer is set when a new target is read in order to stop the robot
		if it is not able to reach the goal.
	"""
	time.sleep(1)
	global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
	global srv_client_go_to_point_, srv_client_wall_follower_, srv_client_user_interface_, pub
	global active_, prev_value_x, prev_value_y

	timeout = None

	rospy.init_node('bug0')

	# subscribing to the topic of the laser and position
	sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	# publisher used for the velocity
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	# services used in the bug0 algorithm
	# go to point - used to reach a point
	# wall follower - used to follow a wall
	srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_service', SetBool)
	srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_service', SetBool)
	
	srv = rospy.Service('bug0_service', SetBool, bug0_switch)

	# initialize target reached
	change_state(2)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if not active_: # if the algorith is not activated, disable go to point
			rate.sleep()
			srv_client_go_to_point_(False)
			prev_value_x = 0
			prev_value_y = 0
			continue
						
		else:
			if regions_ == None:
				continue
	
			# if is set to go to point
			if state_ == 0:
				# compute error position
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,
										2) + pow(desired_position_.x - position_.x, 2))
				
				# if the error is less than 0.3 or the timeout is expired
				if(err_pos < 0.3) or timeout <= rospy.Time.now():
					if timeout <= rospy.Time.now():
						print "Timer Expired!"
					change_state(2) # change the state to target reached

				# if the robot doesn't have an obstacle in front of it
				elif regions_['front'] < 0.5:
					change_state(1) # change the state to go to point

			# if is set to wall follower
			elif state_ == 1:
				# computing the yaw angle and error position
				desired_yaw = math.atan2(
					desired_position_.y - position_.y, desired_position_.x - position_.x)
				err_yaw = normalize_angle(desired_yaw - yaw_)
				err_pos = math.sqrt(pow(desired_position_.y - position_.y,
										2) + pow(desired_position_.x - position_.x, 2))

				# if the error is less than 0.3 or the timeout is expired
				if(err_pos < 0.3) or timeout <= rospy.Time.now():
					if timeout <= rospy.Time.now():
						print "Timer Expired! Target set as"
					change_state(2) # change state to go target reached
				
				# if there is an obstacle in front of the robot
				if regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
					change_state(0) # change the state to go to point

			# if the state is target reached
			elif state_ == 2:
				# check if the new target is already reached
				if prev_value_x != rospy.get_param('des_pos_x') or prev_value_y != rospy.get_param('des_pos_y'):
					# getting the new target from the params
					desired_position_.x = rospy.get_param('des_pos_x')
					desired_position_.y = rospy.get_param('des_pos_y')

					err_pos = math.sqrt(pow(desired_position_.y - position_.y,
										2) + pow(desired_position_.x - position_.x, 2))
					# if the target isn't already reached, start a timer of 40 seconds
					if(err_pos > 0.35):
						print "Starting timer"
						now = rospy.Time.now()
						timeout = now + rospy.Duration(40) # fourty seconds of timeout
						change_state(0) # change the stato to go to point

        rate.sleep()

if __name__ == "__main__":
    main()
