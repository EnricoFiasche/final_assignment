#!/usr/bin/env python

import rospy
import random
from std_srvs.srv import *

## The coordinate x of each target
x_values = [-4.0, -4.0, -4.0, 5.0, 5.0, 5.0]

## the coordinate y of each target
y_values = [-3.0, 2.0, 7.0, -7.0, -3.0, 1.0]

def random_target(empty):
	"""	
		Function used to generate a new random target point.
		Before saving the new target generated, it is checked in order
		to avoid that it is equal to the previous one.
		The coordinates are saved inside the parameters "des_pos_x" and
		"des_pos_y"
		
		Args:
			empty: it is an empty message
	"""
	# reading the previous target from the two parameters
	old_target_x = rospy.get_param("des_pos_x")
	old_target_y = rospy.get_param("des_pos_y")
	new_target_x = old_target_x
	new_target_y = old_target_y
	
	# while the previous target is equal the new one...
	while old_target_x == new_target_x and old_target_y == new_target_y:
		
		# choosing randomly one of the six position
		randomValue = int(random.uniform(0,5))
		new_target_x = x_values[randomValue]
		new_target_y = y_values[randomValue]

	# setting the correct coordinates	
	rospy.set_param("des_pos_x", x_values[randomValue])
	rospy.set_param("des_pos_y", y_values[randomValue])
	
	return []

def main():
	"""
		Main function that define the service random target 
	"""
	rospy.init_node('randomTarget_server') 

	# creating a service related to a function called random_target
	rospy.Service('/randomTarget_service', Empty, random_target)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
