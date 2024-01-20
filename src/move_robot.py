#!/usr/bin/env python3

# Author:	Pablo Tores Rodriguez
# Title:	Pick & Place project
# Date:		20/01/2024

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


#We create a series of variables with wich we are going to communicate with the FSM machine.
picked = False
target = False
start  = False
stop   = False

#We start a few publishers to send the previous variables
start_pub  = rospy.Publisher('start_topic', Bool, queue_size=10)
stop_pub   = rospy.Publisher('stop_topic', Bool, queue_size=10)
picked_pub = rospy.Publisher('picked_topic', Bool, queue_size=10)
target_pub = rospy.Publisher('target_topic', Bool, queue_size=10)

#This variable will keep track of the previous state, this way we avoid reading the same state twice.
prev_state = ""

#We define two commanders for both the arm and the hand.
move_group = moveit_commander.MoveGroupCommander("panda_arm")
eef_group = moveit_commander.MoveGroupCommander("hand")

#Convenience method for testing if the values in two lists are within a tolerance of each other.
def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


#This function will provide the commander with a joint vector to send to the robot.
def go_to_joint_state(posV,moveGroup):
	#we get the current joint position and place ours.
    joint_goal = moveGroup.get_current_joint_values()
    joint_goal = posV.copy()
    print(joint_goal)
    #We order the robot to arrive to the desired joint position.
    moveGroup.go(joint_goal, wait=True)
    #We make sure there is no movement residue.
    moveGroup.stop()
    current_joints = moveGroup.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def callback(data):
		#This function contains all the logic required for the project
        global start,stop,picked,target,prev_state, start_pub,stop_pub,picked_pub,target_pub
        #Check if there has been a change in the state
        if data.data == prev_state:
        	rospy.sleep(1)
        #In the Wait state the machine waits for input.
        elif data.data == "Wait":
                prev_state = data.data
                input("Press enter to start the machine:")
                start = True
                update()
        #In the GOTO state the robot goes to a neutral state awaiting orders.
        elif data.data == "GOTO":
                prev_state = data.data
                go_to_joint_state([0.0,0.444,0.0,-1.5,0.0,1.92,0.778],move_group)
                #If we want to pplace the object we need to move.
                if picked:
                    input("Press enter to place object:")
                    go_to_joint_state([0.0,0.523,0.0,-2.236,0.0,2.732,0.877],move_group)
                #If we want to pick an object we first need to open the fingers and the reach for the object.
                else:
                    input("Press enter to pick object:")
                    go_to_joint_state([0.04,0.04],eef_group)
                    go_to_joint_state([0.001,0.433,0.0,-2.444,0.0,2.529,0.778],move_group)

                target = True
                update()
        #In the PICK state we need to close the fingers and return to the GOTO state.
        elif data.data == "Pick":
                print("Picking object...")
                prev_state = data.data
                go_to_joint_state([0.0,0.0],eef_group)
                picked = True
                target = False
                update()
        #In the PLACE state we need to open the fingers and return to the GOTO state.       
        elif data.data == "Place":
                print("Placing object...")
                prev_state = data.data
                go_to_joint_state([0.04,0.04],eef_group)
                target = False
                picked = False

                        
def update():
        print("Updating info...")
        global start,stop,picked,target, start_pub,stop_pub,picked_pub,target_pub
        start_pub.publish(start)
        stop_pub.publish(stop)
        picked_pub.publish(picked)
        target_pub.publish(target)
        	
   
def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Pick & Place project by Pablo Tores Rodriguez")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin:"
        )
        #Initialize moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        #initialize project node.
        rospy.init_node("pickAndPlaceProject", anonymous=True)
        #Initialize robot commander.
        #rovides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        #Initialize Planning Scene Interface.
        #This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        #Allows to display trajectories in rviz.
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        
        rospy.Subscriber("status", String, callback)
        rospy.spin()
        print("============ Pick & Place project complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
