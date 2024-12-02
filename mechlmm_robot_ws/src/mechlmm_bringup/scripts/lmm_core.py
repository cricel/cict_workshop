#!/usr/bin/env python

import rospy
import tf
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool

import time
import threading
import requests
import base64
import numpy as np

from langchain_core.utils.function_calling import convert_to_openai_function
import lmm_function_pool
import function_pool_lmm_declaration
from function_pool_definition import FunctionPoolDefinition

class DataCommander:
    def __init__(self):
        rospy.init_node('data_commander', anonymous=True)

        self.bridge = CvBridge()
        
        self.counter = 1

        self.base_image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.base_image_callback)
        self.lmm_command_sub = rospy.Subscriber("/mechlmm/command", String, self.mechlmm_command_callback)
        self.cmd_vel_manual_sub = rospy.Subscriber("/cmd_vel/manual", Bool, self.cmd_vel_manual_callback)

        rospy.Timer(rospy.Duration(2), self.timer_callback)

        self.base_processed_image_pub = rospy.Publisher("/camera/rgb/image_raw/detection", Image)

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.lmm_result = None

        self.command_list = []

        self.function_pool_definition = FunctionPoolDefinition()

        self.llm_tools_map = {
            "manipulation": self.function_pool_definition.manipulation,
            "move_robot": self.function_pool_definition.move_robot,
            "idle": self.function_pool_definition.idle,
        }

        
    def cmd_vel_manual_callback(self, _msg):
        if(_msg.data == True):
            self.command_list = []

    def timer_callback(self, msg):
        self.lmm_command_trigger()

    def lmm_command_trigger(self):
        if(len(self.command_list) != 0):

            if (self.latest_frame is None):
                return

            url = 'http://0.0.0.0:5001/mechlmm/chat'

            base_image_url = self.opencv_frame_to_base64(self.latest_frame)

            query = f"""
                    Base on the information and image provided, what are the list of action should do to {self.command_list[-1]}

                    This is a 2 wheel driving robot, the camera where image provided is mount in front of the robot

                    if target is not found, then simply do nothing as idle
                """

            data = {
                'question': query,
                # 'schema': dict_schema,
                'tag': 'head_callback',
                'base_img': [base_image_url],
                'tools': [
                          convert_to_openai_function(function_pool_lmm_declaration.manipulation),
                          convert_to_openai_function(function_pool_lmm_declaration.move_robot),
                          convert_to_openai_function(function_pool_lmm_declaration.idle)
                          ]
            }

            response = requests.post(url, json=data)

            print("-----------------------")
            print(self.counter)
            print("-----------------------")

            if response.status_code == 200:
                _result = response.json()
                print('Success: \n', _result)
                if(_result['type'] == 'tools'):
                    for func in _result['result']:
                        selected_tool = self.llm_tools_map[func['name'].lower()]
                        selected_tool(func['args'])

                    # self.lmm_command_pub.publish("llm send")
            else:
                print('Failed:', response.status_code, response.text)

            self.img_query = False

    def mechlmm_command_callback(self, data):
        self.command_list.append(data.data)
        print(self.command_list)

    def base_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            with self.lock:
                self.counter += 1
                self.latest_frame = cv_image.copy()

        except CvBridgeError as e:
            print(e)

    def image_context_analyzer(self, _frame):
        image_url = self.opencv_frame_to_base64(_frame)

        tag = {"filename": "none"}
        question = "analysis this image, and give me a detail break down of list of objects in the image"
        
        query = {
            "question": question,
            "schema": convert_to_openai_function(lmm_function_pool.ObjectList),
            "base_img": [image_url],
            "tag": tag
        }
        query_result = self.rest_post_request(query)
        
        print("------ image_context_analyzer ------")
        print(query_result)

        return query_result["result"], query_result["tag"]
    
    def rest_post_request(self, _data, _server_url = 'http://' + '0.0.0.0' + ':5001/mechlmm/chat'):
        """
        data = {
            'question': 'question',
            'schema': schema,
            'tag': 'tag',
            'base_img': [base_img_1, base_img_2],
            'tools': [tools_1, tools_2],
            'model': "claude"
        }
        """

        response = requests.post(_server_url, json = _data)

        if response.status_code == 200:
            
            result = response.json()
            return result
        else:
            print('Failed:', response.status_code, response.text)
            return None
        
    def opencv_frame_to_base64(self, _frame):
        _, buffer = cv2.imencode('.jpg', _frame)
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
        image_url = f"data:image/jpeg;base64,{frame_base64}"

        return image_url

    def process_frames(self):
        while True:
            with self.lock:
                if hasattr(self, 'latest_frame'):
                    frame = self.latest_frame
                else:
                    frame = None

            if frame is not None:
                # Analyze the image context
                self.lmm_result, _tag = self.image_context_analyzer(frame)

                # Dictionary to store label colors
                label_colors = {}

                for detected_object in self.lmm_result["objects"]:
                    # Get frame dimensions
                    width, height = frame.shape[1], frame.shape[0]

                    # Calculate bounding box coordinates
                    ymin, xmin, ymax, xmax = detected_object["position"]
                    x1 = int(xmin / 1000 * width)
                    y1 = int(ymin / 1000 * height)
                    x2 = int(xmax / 1000 * width)
                    y2 = int(ymax / 1000 * height)

                    # Assign random color if label is not already in the dictionary
                    label = detected_object["name"]
                    if label not in label_colors:
                        color = np.random.randint(0, 256, (3,)).tolist()
                        label_colors[label] = color
                    else:
                        color = label_colors[label]

                    # Draw the bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                    # Calculate text size and background coordinates
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    font_thickness = 1
                    text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]

                    text_bg_x1 = x1
                    text_bg_y1 = y1 - text_size[1] - 5
                    text_bg_x2 = x1 + text_size[0] + 8
                    text_bg_y2 = y1

                    # Draw text background rectangle
                    cv2.rectangle(frame, (text_bg_x1, text_bg_y1), (text_bg_x2, text_bg_y2), color, -1)

                    # Add the text label
                    cv2.putText(
                        frame, label, (x1 + 2, y1 - 5), font, font_scale, (255, 255, 255), font_thickness
                    )

                try:
                    # Publish the processed image
                    self.base_processed_image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    print(e)


            time.sleep(0.1)

    def run(self):
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down data commander.")

if __name__ == '__main__':
    try:
        node = DataCommander()

        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()