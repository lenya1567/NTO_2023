import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

test_public = rospy.Publisher('test_topic', Image, queue_size=1)

fireRange = ((0, 62, 0), (36, 255, 255))

def image_callback(data):
    global hsvGateMin, hsvGateMax, test_public
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(imgHSV, fireRange[0], fireRange[1])
    fireContours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    fireContours = list(filter(lambda x: cv2.contourArea(x) > 400, fireContours))
    
    i = 0
    for contour in fireContours:
        i += 1
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m00"] / M["m00"])
        print(str(i + 1) + ".", cx, cy)

    test_public.publish(bridge.cv2_to_imgmsg(threshold, 'mono8'))


image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

rospy.spin()