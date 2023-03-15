import rospy
import math
import cv2
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

test_public = rospy.Publisher('test_topic', Image, queue_size=1) # Топик для отладки
fireRange = ((0, 62, 0), (36, 255, 255)) # Границы распознавания цвета пожаров

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

fires = [] # Массив с пожарами (всеми)
        
def image_callback(data):
    global hsvGateMin, hsvGateMax, test_public, fires
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    H, W, _ = img
    imgNew = img[H // 4 : 3 * H // 4, W // 4 : 3 * W // 4] # Изображение уменьшаем для видимости только под коптером
    imgHSV = cv2.cvtColor(imgNew, cv2.COLOR_BGR2HSV) # Изображение переводится в HSV 
    threshold = cv2.inRange(imgHSV, fireRange[0], fireRange[1]) # Преобразование в чёрно-белое изображение через ворота цветов
    fireContours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Поиск контуров
    fireContours = list(filter(lambda x: cv2.contourArea(x) > 600, fireContours)) # Отсечение маленьких контуров

    i = 0
    for contour in fireContours:
        i += 1
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m00"] / M["m00"])
        cv2.drawContours(img, [contour], -1, (255, 0, 0), 5)
        
        T = get_telemetry(frame_id = "aruco_map") # Взятие телеметрии
        x1, y1, z1 = T.x, T.y, T.z

        fires.append([x1, y1]) # Добавление в массив положения пожара 
        print([x1, y1])

    test_public.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))  # Для отладки распознавания пожаров    

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)  

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Передвижение по заданной траектории

navigate(z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

T = get_telemetry(frame_id = "aruco_map")

x0, y0, z0 = T.x, T.y, T.z

navigate_wait(x=0, y=y0, z=1, frame_id="aruco_map")
navigate_wait(x=0, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=4 * 1.005, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=4 * 1.005, y=1 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=4 * 1.005, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=7 * 1.005, y=4 * 1.005, z=1, frame_id="aruco_map")
navigate_wait(x=7 * 1.005, y=0, z=1, frame_id="aruco_map")
navigate_wait(x=x0, y=0, z=1, frame_id="aruco_map")
navigate_wait(x=x0, y=y0, z=1, frame_id="aruco_map")

land()

# Сортировка пожаров

was = []

findFires = []

for e in fires:
    notNeed = False
    for fr in was:
        if get_distance(e[0], e[1], 0, fr[0], fr[1], 0) < 1:
            notNeed = True
            break
    if not notNeed:
        findFires += [(e[0], e[1])]
        was += [(e[0], e[1])]

# Вывод автоматического отчёта

print("Auto report:")

print()
print("Fires:", len(findFires))
print()

for i in range(len(findFires)):
    print("Fire", str(i + 1) + ":", findFires[i][0], findFires[i][1])
