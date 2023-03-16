import rospy
import math
import cv2
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Range
import requests

rospy.init_node("prog")

bridge = CvBridge()

test_public = rospy.Publisher('test_topic', Image, queue_size=1)
fireRange = ((0, 62, 0), (36, 255, 255))
sides = []

publisher = rospy.Publisher("fires", MarkerArray, queue_size=1)
markerArray = MarkerArray()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

fires = []
def addFire(x, y):
    global fires, markerArray, publisher
    notNeed = False
    for fr in fires:
        if get_distance(fr[0], fr[1], 0, x, y, 0) < 0.5:
            notNeed = True
            break
    if not notNeed:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.id = len(markerArray.markers)
        markerArray.markers.append(marker)
        publisher.publish(markerArray)

        URL = "http://65.108.222.51/check_material"
        PARAMS = {'x':round(x, 2), 'y':round(y, 2)}

        r = requests.get(url = URL, params = PARAMS)
        d = r.text.rstrip("\"").lstrip("\"")

        fireClass = "NO"

        if d in ["coal", "textiles", "plastics", "oil", "alcohol", "glycerine"]:
             print("Fire detect right:", d)
             fireClass = d
        else:
             print("Fire detect error", d)

        fires += [(x, y, fireClass)]

        #stream = os.system("curl -X \'GET\' 'http://65.108.222.51/check_material?x=" + str(round(x, 2)) + "&y=" + str(round(x, 2)) + "\' -H \'accept: application/json\'")
        #output = stream.read().rstrip()
        return True
    return False

data = []
def range_callback(msg):
    global data
    dista = msg.range
    T = get_telemetry(frame_id = "aruco_map")
    x, y = T.x, T.y
    if len(data) == 0:
        data.append((dista, x, y))
    last = data[-1]

    if abs(last[0] - dista) > 0.6:
         print(last, last[1], last[2])
         print("Next wall", "wall length", get_distance(x, y, 0, last[1], last[2], 0))
         print("Next wall", "wall length", abs(last[0] - dista))
         data.append((dista, x, y))

def image_callback(data):
    global hsvGateMin, hsvGateMax, test_public, fires, publisher, markerArray
    publisher.publish(markerArray)
    T2 = get_telemetry(frame_id = "aruco_map")
    x1, y1, z1 = T2.x, T2.y, T2.z

    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    H, W, _ = img.shape
    imgNew = img[H // 4 : 3 * H // 4, W // 4 : 3 * W // 4]
    imgHSV = cv2.cvtColor(imgNew, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(imgHSV, fireRange[0], fireRange[1])
    fireContours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    fireContours = list(filter(lambda x: cv2.contourArea(x) > 600, fireContours))

    i = 0
    for contour in fireContours:
        i += 1
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m00"] / M["m00"])
        cv2.drawContours(img, [contour], -1, (255, 0, 0), 5)

        # T2 = get_telemetry(frame_id = "aruco_map")
        #x1, y1, z1 = T.x, T.y, T.z
        if addFire(x1, y1):
            print([x1, y1]) #, get_telemetry(frame_id = "aruco_map"))

    test_public.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

# rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

rospy.Subscriber('rangefinder/range', Range, range_callback)

navigate(z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

T = get_telemetry(frame_id = "aruco_map")

        else:
             print("Fire detect error", d)

        fires += [(x, y, fireClass)]

        #stream = os.system("curl -X \'GET\' 'http://65.108.222.51/check_material?x=" + str(round(x, 2)) + "&y=" + str(round(x, 2)) + "\' -H \'accept: application/json\'")
        #output = stream.read().rstrip()
        return True
    return False

data = []
def range_callback(msg):
    global data
    dista = msg.range
    T = get_telemetry(frame_id = "aruco_map")
    x, y = T.x, T.y
    if len(data) == 0:
        data.append((dista, x, y))
    last = data[-1]

    if abs(last[0] - dista) > 0.6:
         print(last, last[1], last[2])
         print("Next wall", "wall length", get_distance(x, y, 0, last[1], last[2], 0))
         print("Next wall", "wall length", abs(last[0] - dista))
         data.append((dista, x, y))

def image_callback(data):
    global hsvGateMin, hsvGateMax, test_public, fires, publisher, markerArray
    publisher.publish(markerArray)
    T2 = get_telemetry(frame_id = "aruco_map")
    x1, y1, z1 = T2.x, T2.y, T2.z

    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    H, W, _ = img.shape
    imgNew = img[H // 4 : 3 * H // 4, W // 4 : 3 * W // 4]
    imgHSV = cv2.cvtColor(imgNew, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(imgHSV, fireRange[0], fireRange[1])
    fireContours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    fireContours = list(filter(lambda x: cv2.contourArea(x) > 600, fireContours))

    i = 0
    for contour in fireContours:
        i += 1
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m00"] / M["m00"])
        cv2.drawContours(img, [contour], -1, (255, 0, 0), 5)

        # T2 = get_telemetry(frame_id = "aruco_map")
        #x1, y1, z1 = T.x, T.y, T.z
        if addFire(x1, y1):
            print([x1, y1]) #, get_telemetry(frame_id = "aruco_map"))

    test_public.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

# rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

rospy.Subscriber('rangefinder/range', Range, range_callback)

navigate(z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

T = get_telemetry(frame_id = "aruco_map")

x0, y0, z0 = T.x, T.y, T.z

#navigate_wait(x=x0, y=0, z=1, frame_id="aruco_map")
#navigate_wait(x=0, y=0, z=1, frame_id="aruco_map")
navigate_wait(x=0, y=y0, z=1, frame_id="aruco_map")

navigate_wait(x=0, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=0, y=4 * 1.005, z=1, frame_id="aruco_map", yaw=math.pi)
navigate_wait(x=7 * 1.005, y=4 * 1.005, z=1, yaw=math.pi, speed=0.2, frame_id="aruco_map")


'''navigate_wait(x=4 * 1.005, y=1 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=4 * 1.005, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=7 * 1.005, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=7 * 1.005, y=0, z=1, frame_id="aruco_map")
navigate_wait(x=x0, y=0, z=1, frame_id="aruco_map")
navigate_wait(x=x0, y=y0, z=1, frame_id="aruco_map")
'''
land()

#rospy.sleep(10)
'''
was = []

findFires = []

for e in fires:
    notNeed = False
    for fr in was:
        if get_distance(e[0], e[1], 0, fr[0], fr[1], 0) < 0.5:
            notNeed = True
            break
    if not notNeed:
        findFires += [(e[0], e[1])]
        was += [(e[0], e[1])]
'''
print("Auto report:")

print()
print("Fires:", len(fires))
print()

for i in range(len(fires)):
    name = fires[i][2]
    group = "undefined"
    if name in ["coal", "textiles", "plastics"]:
        group = "A"
    if name in ["oil", "alcohol", "glycerine"]:
        group = "B"
    print("Fire", str(i + 1) + ":", round(fires[i][0], 2), round(fires[i][1], 2), name, group)

print("Injured: 0")

c = 0
for i in range(len(data)):
    c += 1
    print("Wall", str(c) + ":", get_distance(x, y, 0, last[1], last[2], 0))

    c += 1
    print("Wall", str(c) + ":", abs(last[0] - dista))
