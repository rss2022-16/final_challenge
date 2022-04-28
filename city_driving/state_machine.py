#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Normal
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_sign','car_wash', 'exit'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Normal')
        # run line_follower
        # run safety_controller
        # run stop sign detector
        # run car wash detector
        if stop_sign_detected():
            return 'stop_sign'
        elif car_wash_detected():
            return 'car_wash'
        elif exited_city():
            return 'exit'


# define state Stop_sign
class Stop_sign(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_sign')
        # stop car 
        return 'normal'


# define state Car_wash
class Car_wash(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['normal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Car_wash')
        # drive towards blue streamers
        # disable safety_controller and line_follower
        # run wall_follower 
        return 'normal'
        


def main():
    rospy.init_node('city_driving_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['complete'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Normal', Normal(), 
                               transitions={'stop_sign':'Stop_sign', 'car_wash':'Car_wash', 'exit': 'complete'})
        smach.StateMachine.add('Stop_sign', Stop_sign(), 
                               transitions={'normal':'Normal'})
        smach.StateMachine.add('Car_wash', Car_wash(), 
                               transitions={'normal':'Normal'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()