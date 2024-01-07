#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def callback(in_message : CompressedImage):
    """
    Converts the received CompressedImage message into a Image message
    and publishes it into the topic for ORBSLAM3 node to read
    """

    # Convert the compressed image message to an OpenCV image
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(in_message, desired_encoding="passthrough")

    # Convert the OpenCV image to an Image message
    out_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    # # Publish Image for SLAM algorithm to receive
    # out_msg = Image()
    # out_msg.header.stamp = in_message.header
    # out_msg.height = 480
    # out_msg.width = 640
    # out_msg.encoding = "rgb8"
    # out_msg.is_bigendian = False
    # out_msg.step = 3 * 
    # out_msg.data = 

    pub.publish(out_msg)


if __name__ == "__main__":

    # Create nodes
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/pato/camera_node/image/compressed', CompressedImage, callback)
    rospy.spin()
