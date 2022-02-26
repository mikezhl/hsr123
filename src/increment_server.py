import rospy
from std_srvs.srv import Empty,EmptyResponse

def increment(req):
    n = int(rospy.get_param("pickup_status"))+1
    rospy.set_param('pickup_status', n)
    print("Increment: ",n+1)
    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("Increment")
    server = rospy.Service("Increment",Empty,increment)
    rospy.spin()