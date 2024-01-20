#!/usr/bin/env python3

import rospy
import time
import smach
import smach_ros
from std_msgs.msg import Float32, String, Bool

# Global Variable 
counter_change = 0
picked = False
target = False
stop = False
start = False
pub = rospy.Publisher('/status', String, queue_size=1000) 

# define state Foo
class WAIT(smach.State):
    def __init__(self):            # outcome della macchina a stati
        smach.State.__init__(self, outcomes=['home_pose','wait']) #

    def execute(self, userdata):
        global start, stop
        rospy.loginfo('Executing state Locked')
        pub.publish('Wait')
        time.sleep(2)
        if start == True and stop == False:
            return 'home_pose'
        else:
            return 'wait'


# define state Bar
class GOTO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait','place','pick','stop'])

    def execute(self, userdata):
        global target, picked, stop, start

        time.sleep(2)

        start = False

        pub.publish('GOTO')
        if stop == True:
            
            return 'stop'
        
        elif target == False:

            return 'wait'
        
        elif picked == False:

            return 'pick'
        
        else: 
            
            return 'place'

class PICK(smach.State):
    def __init__(self):            # outcome della macchina a stati
        smach.State.__init__(self, outcomes=['wait','target_position','stop']) #

    def execute(self, userdata):
        global picked, target, stop

        time.sleep(2)

        pub.publish('Pick')
        if stop == True:

            return 'stop'
        
        elif picked == True:

            return 'target_position'
        
        else:

            return 'wait'
        

class PLACE(smach.State):
    def __init__(self):            # outcome della macchina a stati
        smach.State.__init__(self, outcomes=['wait','home_position','stop']) #

    def execute(self, userdata):
        global picked, target, stop
        rospy.loginfo('Executing state Locked')
        pub.publish('Place')

        time.sleep(2)
        
        if stop == True:

            return 'stop'
        
        elif picked == True:
            
            target = False
            picked = False
            return 'home_position'
        
        else:

            return 'wait'


# NODO ROS
def Callback_start(data):
    global start
    rospy.loginfo("I heard %s", data.data)
    start = data.data

def Callback_stop(data):
    global stop
    rospy.loginfo("I heard %s", data.data)
    stop = data.data

def Callback_picked(data):
    global picked
    rospy.loginfo("I heard %s", data.data)
    picked = data.data

def Callback_target(data):
    global target
    rospy.loginfo("I heard %s", data.data)
    target = data.data

def State_Machine():

    rospy.init_node('state_listner', anonymous= True)
    rospy.Subscriber("start_topic",  Bool, Callback_start)
    rospy.Subscriber("stop_topic",   Bool, Callback_stop)
    rospy.Subscriber("picked_topic", Bool, Callback_picked)
    rospy.Subscriber("target_topic", Bool, Callback_target)

    #rospy.spin()

def main():

    State_Machine()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[]) 

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT', WAIT(), 
                               transitions={'wait':'WAIT', 
                                            'home_pose':'GOTO'})
        

        smach.StateMachine.add('GOTO', GOTO(), 
                               transitions={'stop':'WAIT',
                                            'wait':'GOTO',
                                            'pick':'PICK',
                                            'place':'PLACE'})
        
        smach.StateMachine.add('PLACE', PLACE(), 
                               transitions={'stop':'WAIT',
                                            'wait':'PLACE',
                                            'home_position':'GOTO'})

        smach.StateMachine.add('PICK', PICK(), 
                               transitions={'stop':'WAIT',
                                            'wait':'PICK',
                                            'target_position':'GOTO'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/Pick_and_Place_FSM')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
