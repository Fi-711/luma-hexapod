#!/usr/bin/env python3
# Modified version of core electronics script, converted to ROS2 and added own logic: https://core-electronics.com.au/guides/object-identify-raspberry-pi/

"""
Modified script of Josh Newans
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge              import CvBridge, CvBridgeError
import numpy as np
import cv2


class DetectPerson(Node):
    #This is to pull the information about what each object is called
    classNames = []
    classFile = "/home/pi-ubuntu/ros2_hexapod/detect_person/coco.names"
    with open(classFile,"rt") as f:
        classNames = f.read().rstrip("\n").split("\n")

    #This is to pull the information about what each object should look like
    configPath = "/home/pi-ubuntu/ros2_hexapod/detect_person/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    weightsPath = "/home/pi-ubuntu/ros2_hexapod/detect_person/frozen_inference_graph.pb"
    net = cv2.dnn_DetectionModel(weightsPath, configPath)
    net.setInputSize(320,320)
    net.setInputScale(1.0/ 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)


    def __init__(self, compression_quality=0):
        super().__init__('detect_person')
        self.compression_quality = compression_quality
        self.get_logger().info('Looking for the person...')
        # self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        # self.hexapod_camera_subscriber_ = self.create_subscription(Image,"/hexapod_camera_topic",self.camera_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.hexapod_camera_compressed_subscriber_ = self.create_subscription(CompressedImage,"/hexapod_camera_compressed_topic",self.camera_compressed_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_person_out", 1)
        # self.image_compressed_out_pub = self.create_publisher(CompressedImage, "/image_compressed_out", 1)
        # self.image_compressed_tuning_pub = self.create_publisher(CompressedImage, "/image_compressed_tuning", 1)
        self.person_pub  = self.create_publisher(String,"/detected_person",10)
        
        # set up values 

        self.bridge = CvBridge()

    def getObjects(self, img, thres, nms, draw=True, objects=[]):
        classIds, confs, bbox = self.net.detect(img,confThreshold=thres,nmsThreshold=nms)
        #print(classIds,bbox)
        if len(objects) == 0: objects = self.classNames
        objectInfo =[]
        if len(classIds) != 0:
            for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
                className = self.classNames[classId - 1]
                if className in objects:
                    objectInfo.append([box,className])
                    if (draw):
                        cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                        cv2.putText(img, self.classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                        cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)

        return img, objectInfo

    def camera_compressed_callback(self,img_msg):
        try:
            # decompress the image
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            result, objectInfo = self.getObjects(img,0.45,0.5, objects=['person'])
            # print(objectInfo)
            # print(result)

            cv2.imshow("Output",img)
            cv2.waitKey(1)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     cv2.destroyAllWindows()

            # img_to_pub = self.bridge.cv2_to_imgmsg(img, "bgr8")
            # img_to_pub.header = img_msg.header
            # self.image_out_pub.publish(img_to_pub)

            # if a person found publish it
            msg_to_pub = String()
            if len(objectInfo) > 0:
                msg_to_pub.data = "person_found"
            else:
                msg_to_pub.data = ""
            
            self.person_pub.publish(msg_to_pub)

            # # OUT IMAGE
            # # compress the image using PNG - 0 = max compression, 9 = least compression
            # success, encoded_image = cv2.imencode('.png', out_image, [cv2.IMWRITE_PNG_COMPRESSION, self.compression_quality])

            # # check if compression was successful
            # if not success:
            #     self.get_logger().error('Failed to compress image')

            # # create compressed image message
            # msg = CompressedImage()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.format = 'png'
            # msg.data = encoded_image.tobytes()

            # # publish the message
            # self.image_compressed_out_pub.publish(msg)


        except CvBridgeError as e:
            print(e)  

def main(args=None):
    rclpy.init(args=args)
    node = DetectPerson()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
