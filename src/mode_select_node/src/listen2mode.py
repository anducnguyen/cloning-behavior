
from __future__ import print_function

import sys
import rospy

def listened_mode(x):
    rospy.wait_for_service('mode_pub')
    try:
        listened_mode = rospy.ServiceProxy('mode_pub')
        resp1 = listened_mode(x):
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting Mode"
    print("Current mode is%s"%(xxs))