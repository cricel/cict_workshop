#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput.keyboard import Listener


class KeyboardInputPublisher:
    def __init__(self):
        rospy.init_node('keyboard_input_publisher', anonymous=True)

        self.keyboard_pub = rospy.Publisher('keyboard', String, queue_size=10)

        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        rospy.loginfo("Keyboard input publisher initialized.")
        rospy.loginfo("Use Ctrl+C to exit.")

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                rospy.loginfo(f"Key pressed: {key.char}")
                self.keyboard_pub.publish(key.char)
            else:
                rospy.loginfo(f"Special key pressed: {key}")
                self.keyboard_pub.publish(str(key))
        except AttributeError:
            rospy.logwarn(f"Error with key: {key}")

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down keyboard input publisher.")
        self.listener.stop()

if __name__ == '__main__':
    try:
        node = KeyboardInputPublisher()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()