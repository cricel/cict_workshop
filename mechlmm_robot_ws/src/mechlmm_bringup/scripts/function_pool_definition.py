#!/usr/bin/env python
import sys
import cv2
import time
import threading

import rospy

import moveit_commander
from geometry_msgs.msg import Pose

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped, Point, Pose, Quaternion, PoseStamped


class FunctionPoolDefinition:
    def __init__(self):
        self.robot_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lmm_cmd_publisher = rospy.Publisher('/base_cmd/input/lmm', Twist, queue_size=10)

        self.mux_input = False

        rospy.sleep(0.5)

    def move_robot(self, _args):
        thread = threading.Thread(target=self.robot_cmd_thread, args=(_args["direction"],))
        thread.start()

    def robot_cmd_thread(self, _direction, _distance = 0.3, _rotation = 0.3):
        msg = Twist()

        if(_direction == "forward"):
            msg.linear.x = _distance
            
        elif(_direction == "turn_left"):
            msg.angular.z = _rotation

        elif(_direction == "turn_right"):
            msg.angular.z = -_rotation

        elif(_direction == "backward"):
            msg.linear.x = -_distance

        elif(_direction == "stop"):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if(self.mux_input):
            self.lmm_cmd_publisher.publish(msg)
        else:
            self.robot_cmd_publisher.publish(msg)

            time.sleep(1)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.robot_cmd_publisher.publish(msg)
        
        return msg

    def navigation(self, _arg):
        print("trying to do navigation")
    
    def manipulation(self, _arg):
        print("trying to do manipulation")

    def idle(self, _arg):
        print("idle")

def main(args):
    rospy.init_node('function_pool_definition', anonymous=True)

    fpd = FunctionPoolDefinition()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)