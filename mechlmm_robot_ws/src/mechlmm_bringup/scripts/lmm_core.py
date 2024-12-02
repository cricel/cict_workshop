#!/usr/bin/env python

import rospy
import tf
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image

import time
import threading
import requests
import base64

from langchain_core.utils.function_calling import convert_to_openai_function
import lmm_function_pool

class DataCommander:
    def __init__(self):
        rospy.init_node('data_commander', anonymous=True)

        self.bridge = CvBridge()
        
        self.base_image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.base_image_callback)

        self.base_processed_image_pub = rospy.Publisher("/camera/rgb/image_raw/detection", Image)

        self.lock = threading.Lock()
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.lmm_result = None

    def base_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            with self.lock:
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
                self.lmm_result, _tag = self.image_context_analyzer(frame)

                for detected_object in self.lmm_result["objects"]:
                    # Calculate bounding box coordinates
                    ymin, xmin, ymax, xmax = [int(coord / 1000 * frame.shape[0 if j % 2 == 0 else 1]) for j, coord in enumerate(detected_object["position"])]
                    # Draw rectangle
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    
                    # Add text label
                    cv2.putText(frame, detected_object["name"], (ymin, xmin + 2),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.9, 
                                (255, 0, 0), 
                                2)

                try:
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