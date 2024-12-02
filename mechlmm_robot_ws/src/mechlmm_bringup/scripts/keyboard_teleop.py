#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class KeyboardCommander:
    def __init__(self):
        rospy.init_node('operator_arm_manual_control', anonymous=True)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_manual_pub = rospy.Publisher('/cmd_vel/manual', Bool, queue_size=10)

        self.keyboard_sub = rospy.Subscriber("/keyboard",String, self.keyboard_callback)
        
        self.base_linear_speed = 0.1
        self.base_rotate_speed = 0.3

    def keyboard_callback(self, _msg):
        self.base_key_control(_msg)

    def base_key_control(self, _msg):
        move_cmd = Twist()

        if(_msg.data == 'w'):
            move_cmd.linear.x = self.base_linear_speed
            
        elif(_msg.data == 'a'):
            move_cmd.angular.z = self.base_rotate_speed

        elif(_msg.data == 'd'):
            move_cmd.angular.z = -self.base_rotate_speed

        elif(_msg.data == 's'):
            move_cmd.linear.x = -self.base_linear_speed

        elif(_msg.data == 'x'):
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        else:
            return

        self.cmd_pub.publish(move_cmd)

        self.cmd_manual_pub.publish(True)

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down keyboard input publisher.")

if __name__ == '__main__':
    try:
        node = KeyboardCommander()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()