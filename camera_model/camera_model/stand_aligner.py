#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class StandAligner(Node): #create the class and inharet from node
    def __init__(self): 
        super().__init__("camera_model") #call the init from the Node class
        # this is the subscriber to the camera 
        self.camera_image_sub_ = self.create_subscription(Image,'/camera/image_raw',self.image_box_detection,10)
        #this is the publisher fot the geo msg twist 
        self.twist_vel_pub_ = self.create_publisher(Twist,"cmd_vel",10)
        # this is the converter from the camera image to cv image that the model accepts 
        self.bride_converter_to_cv_ = CvBridge()

        self.the_trained_model_ = YOLO("type/the_path/to/the/model/best.pt") #here you will type the path to the model we trained already on detecting the slot


        #this parameters is not the final thing look on the values we need to send to the esp ****important******    
        # Control parameters
        self.k_linear = 0.002   # proportional constant for forward/back speed
        self.k_angular = 0.005  # proportional constant for rotation    

        self.get_logger().info("the model node is detecting.....")


    def image_box_detection(self, image : Image):

        #converting the image to a frame accpted to the model 
        Frame_ = self.bride_converter_to_cv_.imgmsg_to_cv2(image , "bgr8")


        image_result_ = self.the_trained_model_(Frame_)

        slots_boxs_ = image_result_[0].boxes.xyxy.cpu().numpy()

        if len(slots_boxs_) > 0:
            x1,y1,x2,y2 = slots_boxs_[0]

            cx = (x1 + x2 ) / 2
            cy = (y1 + y2 ) / 2

            error_x = (cx - Frame_.shape[1]) / 2
            error_y = (cy - Frame_.shape[0]) / 2

            #takecare of what you sends to the esp in any form

            twist_= Twist()

            twist_.linear.x  = self.k_linear * error_y
            twist_.angular.z = -self.k_angular * error_x 

            self.twist_vel_pub_.publish(twist_)
        else:
            pass


        



def main(args=None):
    rclpy.init(args=args)   #init the communication
    node=StandAligner()   #create a object from the class
    rclpy.spin(node)        #make the node spin
    rclpy.shutdown()        #shutdow the node



#if the name main called in the terminal call the main function
if __name__ == "__main__":
    main()