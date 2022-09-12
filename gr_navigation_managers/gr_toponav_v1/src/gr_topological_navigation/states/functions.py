import rospy

def update_cb(userdata, msg):
    #False/None to terminate
    #True valid
    userdata.execution_requested = True
    rospy.logwarn("message received")
    return "valid"
