#!/usr/bin/env python

'''Node to send /setpoint message, 0.2 and -0.2 per 5 seconds'''

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

def run():
    '''main'''
    # init node
    rospy.init_node('pid_test_setpoint')
    # init publish topic
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub_duty = rospy.Publisher('duty', Float64MultiArray, queue_size=10)
    # main loop
    rate = rospy.Rate(100)
    desired_speed = 0.05
    msg = Twist()
    duty_msg = Float64MultiArray()
    while not rospy.is_shutdown():
        for i in range(200):
            msg.linear.x = desired_speed
            duty_msg.data = [desired_speed / 2, -desired_speed / 2]
            pub_cmd_vel.publish(msg)
            # pub_duty.publish(duty_msg)
            rate.sleep()
        desired_speed = -desired_speed

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
