#! /usr/bin/env python
# coding:utf-8
import rospy, rosparam
import numpy as np
import cv2
from sensor_msgs.msg import Image
from darknet_dnn.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError


class DrawBoundingBox():
    def __init__(self):
        self.sub_img_name = rospy.get_param("sub_img_name", "/usb_cam/image_raw")
        self.sub_img = rospy.Subscriber(self.sub_img_name, Image, self.sub_img_CB)

        self.pub_bbox_name = rospy.get_param("target_boundingBox_name", "/tracker_ros/update_bbox")
        self.pub_bbox = rospy.Publisher(self.pub_bbox_name, BoundingBox, queue_size=1, latch=False)

        self.input_img = Image()
        self.drawing = False
        self.ix, self.iy = -1, -1

    def sub_img_CB(self, msg):
        try:
            self.input_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            rospy.logerr(str(e))


    def draw_bbox(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            #print "push"
            self.drawing = True
            self.ix, self.iy = x, y

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                #print "moving..."
                cv2.rectangle(self.input_img, (self.ix,self.iy),(x,y),(0,255,0), 2)

        elif event == cv2.EVENT_LBUTTONUP:
            #print "up"
            self.drawing = False
            cv2.rectangle(self.input_img, (self.ix,self.iy),(x,y),(0,255,0), 2)
            bbox = BoundingBox()
            bbox.x = self.ix
            bbox.y = self.iy
            bbox.width = x - self.ix
            bbox.height = y - self.iy
            self.pub_bbox.publish(bbox)

    def main(self):
        window_name = "DrawBoundingBox[%s]"%self.sub_img_name
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.draw_bbox)

        while not rospy.is_shutdown():
            try:
                cv2.imshow(window_name, self.input_img)
                cv2.waitKey(1)
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break
            except Exception as e:
                #rospy.logerr(str(e))
                pass

if __name__ == "__main__":
    rospy.init_node("draw_boundingbox")

    db = DrawBoundingBox()
    db.main()
    rospy.spin()
