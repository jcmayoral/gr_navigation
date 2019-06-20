from sensor_msgs.msg import Image, CameraInfo
import rospy
import time


rospy.init_node("synchronizer")
pub = rospy.Publisher("/camera/depth_syncro/image_rect_raw", Image, queue_size=10)
pub2 = rospy.Publisher("/camera/depth_syncro/camera_info", CameraInfo, queue_size=10)
img = Image()
caminfo = CameraInfo()
local_seq = 0
last_time = rospy.Time.now()

def syncronizer(msg, is_img):
    global img, caminfo, last_time
    last_time = rospy.Time.now()
    if is_img:
        img = msg
    else:
        caminfo = msg

def timer_cb(event):
    global pub, pub2, img, caminfo, local_seq, last_time
    caminfo.header.seq = local_seq
    caminfo.header.stamp = rospy.Time.now()
    img.header.stamp = rospy.Time.now()
    pub.publish(img)
    pub2.publish(caminfo)
    local_seq = local_seq + 1
    if (rospy.Time.now().to_sec() - last_time.to_sec() > 0.2):
        local_seq = 0




rospy.Subscriber("/camera/depth/image_rect_raw", Image, syncronizer, True)
rospy.Subscriber("/camera/depth/camera_info", CameraInfo, syncronizer, False)

#rospy.Timer(rospy.Duration(0.1), timer_cb)

while not rospy.is_shutdown():
    time.sleep(0.1)
    timer_cb(0)
#rospy.spin()
