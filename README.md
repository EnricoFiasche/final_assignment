# Research Track I - Final Assignment S4482512 Enrico Fiasche'

## General information
The assignment requires developing a software architecture for the control of the robot in the environment. The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion.

The simulation can be launched by the executing the command:

```
roslaunch final_assignment final.launch
```

### The simulation can perform the following behaviours:
- 1. Move the robot randomly in the environment, by choosing one out of six possible target positions;
- 2. Ask the user for the next target position (checking that the position is one of the possible six) and reach it;
- 3. Start following the external walls;
- 4. Stop the robot in the last position;
- 5. Change the planning algorithm to Dijkstra to bug0 and vice versa.

If the robot is in state 1, or 2, the system should wait until the robot reaches the position in order to swtich to the state 3, 4 or 5.

## Nodes and Programs descriptions
I have diffent programs used to achive the behaviours written above. The programs are written in Python and they are:
- The first one, **go_to_point_service_m.py**, is a service server used to go straight to the objective. This service is used when the robot has correct heading but it is away from the desired point. This service is used by the __bug0 algorithm__ to reach the target point.
- The second one, **wall_follow_service_m.py**, is a service server used to follow the wall. This service first of move the robot in a anti-clockwise circle until the wall is found. Then it reachs the wall and with the front-right obstructed the robot will follow the wall.
This service is used to achive the point 3 (follow the wall) and is used on the _bug0 algorithm_ to help to reach the target.
- The third one, **bug_m.py**, is a service server used to reach a given target point using the __bug0 algorithm__.
This program starts as disabled with state "target reached" and with the service __go to point service__ disabled. When it is activated, by the __control__ program, the program waits a new target in the parameter _des_pos_x_ and _des_pos_y_.
When a new target is given the program starts a timer of 40 seconds, because not all target can be reached using this algorithm. It works using a simple state machine with three state:
    * Go to point: denotes the state when the robot has correct heading but is away from the target point by a distance greater than some threshold (it is permormed by the service __go_to_point__)
    * Wall follower: denotes the state when robot heading differs from the desired heading by more than a threshold (it is performed by the service __wall_follow__)
    * Target reached: denotes the state when the robot has correct heading and has reached the destination. In this state the velocity is set to zero and it communicates to the __control__ program that the goal is reached.
- The fourth program, **userCommand_server.py**, is a service server that, when it is active, allows the user to choose a command among five possible explained above. This service checks also if the new command is equal to the previous one, in order to avoid that the function is vainly repeated several time. If the new command is valid, it is saved inside a parameter _command_ in order to be read in the __control__ program.
- The fifth program, **userTarget_server.py**, is a service server that allows the user to choose a new target between the six printed on the screen, this service is called when the user wants to execute the first command, so when the command is equal to one. The new target coordinates are saved inside two parameters, _des_pos_x_ and _des_pos_y_, in order to be read in the __control__ program.
- The sixth program, **randomTarget_server.py**, is a service server that generates a random target among six possible goals, this service is called when the user wants to execute the second command, so when the command is equal to two. The new target coordinates are saved inside two parameters, _des_pos_x_ and _des_pos_y_, in order to be read in the __control__ program.
- The last program, **controller.py**, manages the behaviours of the robot. This program uses all the previous node services in order to perform correctly all the behaviours written above. It subscribes also to the topic _move_base/result_ to understand when the robot change its status, from _ACTIVE_ to _SUCCEEDED_ (target reached). It also has two publisher used to pubish the goal for __move_base__ and to publish the velocity through __cmd_vel__.
This program has an endless loop, where it reads each time the value of the __command__ inside a parameter. Based on the behaviours permormed by the robot explained before, the __controller__ activates and disables the correct service taking in account if the bug0 is activated. If the bug0 isn't activated the new target is set calling the services interested and using the topic _move_base/goal_ in order to allow the __move_base algorithm__ to achive the objective, else if bug0 is activeted the program calling only the services interested, because inside this algorithm there is a part where it get the coordinates from the parameters.

## Content packages
The folder _final_assignment_ contains launch files that allow the terminal to execute all nodes toghether, scripts files where are written the codes and the description of the world and the robot.
    <br/>&nbsp;&nbsp;&nbsp;&nbsp;.<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ CMakeLists.txt<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_config<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ sim.riviz<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ launch<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ final.launch<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ move_base.launch<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ simulation_gmapping.launch<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ package.xml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ param<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ base_local_planner_params.yaml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ costmap_common_params.yaml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ global_costmap_params.yaml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ local_costmap_params.yaml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ move_base_params.yaml<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ rosgraph.svg<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ scripts<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ bug_m.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ controller.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ go_to_point_service_m.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ randomTarget_server.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ userCommand_server.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ userTarget_server.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ wall_follow_service_m.py<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ tree.png<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ urdf<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;&nbsp;|_ robot.urdf<br/>
    &nbsp;&nbsp;&nbsp;&nbsp;|_ worlds<br/><br/>

Inside the final.launch there are all the nodes useful to start the simulation and to perform the given behaviours.
