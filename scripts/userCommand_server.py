#!/usr/bin/env python

import rospy
from std_srvs.srv import *

## Global variables that checks if the service is active
active_ = False

def user_command(request):
	"""
		Function used to enable the service.
		
		Args:
			request - it contains the value used to enable the service,
					  it could be True or False.
	"""
	global active_
    
	active_ = request.data
	result = SetBoolResponse()
	result.success = True
	result.message = 'Done!'

	return result
	
def change_command():
	"""
		Function used to get the command chosen by the user.
		
		First of all it disable this service in order to avoid that the
		function is vainly repeated several times.
		
		It checks also if the command is already active, if it is true
		the function doesn't set the command and ask again the command.
		The chosen command is set on a parameter called "command".
	"""	
	global active_

	active_ = False # disabling the service
	
	# reading the previous choice
	oldChoice = rospy.get_param('/command')
	choice = oldChoice
	
	# while the previous choice is equal to the new one, or
	# while the choice is equal to an unknown command
	while (oldChoice == choice) or (choice > 5 or choice < 1):
		if choice > 5 or choice < 1: # print an error message
			print "Unknown command, please try again"

		print("Please select one of the following senteces\n")
		print("1 - Move the robot randomly, by choosing one of six possible target point\n")
		print("2 - The user can type the next target\n")
		print("3 - Start following the external walls\n")
		print("4 - Stop the robot in the last position\n")
		print("5 - Change the planning algorithm from move_base to bug0 and vice versa\n")

		# read the command chosen by the user	
		choice = (int(raw_input("Please select a number between 1 and 5: ")))

	# set the chosen command
	if choice >= 1 and choice <= 5:
		rospy.set_param('/command', choice)

def main():
	"""	
		Main function that allows the user to type the next new target point.
		Each time it checks if the service is active, if it is false check also
		if the current command is two (user target) and change it in the command
		four (stop the robot).
		If the service is active it call the function change_command that allows
		the user to choose a new command.
	"""
	global active_
	
	rospy.init_node('userCommand_server')

	# service that allows the user to choose a new command
	srv_userCommand = rospy.Service('/userCommand_service', SetBool, user_command)
	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		if not active_: # if the service is disabled
			rate.sleep()
			
			# if the actual command is two 
			if rospy.get_param("/command") == 2:
				rospy.set_param("/command",4) # change it in the command four
			
			continue
			
		else: # if the service is active
			change_command() # allow the user to choose a new command
		
		rate.sleep()
		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
