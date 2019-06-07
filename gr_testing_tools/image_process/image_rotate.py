#/usr/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

global publisher
global bridge

def msg_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cv_image.shape
    M = cv2.getRotationMatrix2D((rows/2, cols/2), 180, 1)
    cv_image = cv2.warpAffine(cv_image, M, (rows, cols))

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
        publisher.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node("rotate_image_node_for_testing")
    publisher = rospy.Publisher("rotated_image", Image, queue_size=1)
    rospy.Subscriber("camera/color/image_raw", Image, msg_callback, queue_size=1)
    rospy.spin()
    
