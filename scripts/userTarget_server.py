#!/usr/bin/env python

import rospy
from std_srvs.srv import *

## The coordinate x of each target 
x_values = [-4.0, -4.0, -4.0, 5.0, 5.0, 5.0]

## The coordinate y of each target
y_values = [-3.0, 2.0, 7.0, -7.0, -3.0, 1.0]

def user_target(request):
	"""	
		Function used to allow the user to choose a new target.
		It checks if the chosen target is already reached (checking if
		the new one is equal to the previous one).
		
		If the new target is correct and it is different from the
		previous one, the program writes the two coordinates on the
		parameters "des_pos_x" and "des_pos_y"
	"""
	
	# getting the previous target
	old_target_x = rospy.get_param("des_pos_x")
	old_target_y = rospy.get_param("des_pos_y")
	new_target_x = old_target_x
	new_target_y = old_target_y

	choice = 0

	# while the new target is equal to the previous one or
	# while the target chosen is an unknown one.
	while (old_target_x == new_target_x and old_target_y == new_target_y) or (choice > 6 or choice < 1):
		# target options
		print "Choose one of these options:"
		print "1 - [-4.0,-3.0]\n2 - [-4.0, 2.0]\n3 - [-4.0, 7.0]\n4 - [5.0, -7.0]\n5 - [5.0, -3.0]\n6 - [5.0, 1.0]"
		
		# print the previous target, if exist
		if old_target_x != 0 and old_target_y != 0:
			print "The previous target was [" + str(old_target_x) + ", " + str(old_target_y) + "]"
			
		choice = int(raw_input("\nUser Target: Choose a number from 1 to 6: "))
		
		# check if it the target chosen is valid
		if choice <= 6 and choice >= 1:
			new_target_x = x_values[choice-1]
			new_target_y = y_values[choice-1]

	# setting the correct coordinates
	rospy.set_param("des_pos_x", x_values[choice-1])
	rospy.set_param("des_pos_y", y_values[choice-1])

	# display the random target
	print ("Target position [" + str(x_values[choice-1]) + ", " + str(y_values[choice-1]) + "]\n")
	
	return []

def main():
	"""	
		Main function that define the userTarget service
	"""
	
	rospy.init_node('userTarget_server')
	
	# service used to allow the user to choose a new target
	rospy.Service('/userTarget_service', Empty, user_target)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

