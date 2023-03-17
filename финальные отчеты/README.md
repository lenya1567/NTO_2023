# Финальный отчёт команды "Rакета67"
'''python3
    # Импортирование библиотек
    import rospy
    import math
    import cv2
    import time
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

    # Создание топиков
    fire_topic = rospy.Publisher('fire_topic', Image, queue_size=1)
    people_topic = rospy.Publisher('people_topic', Image, queue_size=1)

    # Рамки для распознавания цветов
    fireRange = ((0, 62, 0), (36, 255, 255))
    peopleRange = ((102, 102, 0), (110, 255, 255))

    # Топик и массив объектов для визуализации в ROS3dJS
    publisher = rospy.Publisher("fires", MarkerArray, queue_size=1)
    markerArray = MarkerArray()

    # Топик для просмотра телеметрии
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

    # Время и данные последней телеметрии
    lastTelem = None
    lastTime = time.time()

    # Коптер дошёл до старта опознавания стен
    wallStart = False

    # Функция для взятия телеметрии
    def getTelem():
        global lastTelem, lastTime

        T = time.time()
        if T - lastTime > 0.05:
            lastTime = T
            lastTelem = get_telemetry(frame_id = "aruco_map")
        return lastTelem

    # Расстояние между точками в 3D
    def get_distance(x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    # Полёт коптера в точку с ожиданием
    def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
        navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    # Огни и функция для их добавления и запроса к серверу
    fires = []
    def addFire(x, y):
        global fires, markerArray, publisher, wallStart
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

            return True
        return False

    # Функция для измерения длины стен
    data = []
    def range_callback(msg):
        global data, wallStart
        if not wallStart: return
        dista = msg.range
        T = getTelem()
        x, y = T.x, T.y
        if len(data) == 0:
            data.append((dista, x, y))
        last = data[-1]

        if abs(last[0] - dista) > 0.6:
             print(last, last[1], last[2])
             print("Next wall", "wall length", get_distance(x, y, 0, last[1], last[2], 0))
             print("Next wall", "wall length", abs(last[0] - dista))
             data.append((dista, x, y))

    # Проверка на объект (пожар или человека)
    def checkForObject(img, color, cnt):
        H, W, _ = img.shape
        imgNew = img[H // 4 : 3 * H // 4, W // 4 : 3 * W // 4]
        imgHSV = cv2.cvtColor(imgNew, cv2.COLOR_BGR2HSV)
        threshold = cv2.inRange(imgHSV, color[0], color[1])
        fireContours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        fireContours = list(filter(lambda x: cv2.contourArea(x) > 600, fireContours))

        i = 0
        for contour in fireContours:
            i += 1
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m00"] / M["m00"])
            cv2.drawContours(img, [contour], -1, cnt, 5)

            # T2 = get_telemetry(frame_id = "aruco_map")
            #x1, y1, z1 = T.x, T.y, T.z
            return True

        return False

    # Функция для обработки фотографии, полученной из топика
    def image_callback(data):
        global hsvGateMin, hsvGateMax, test_public, fires, publisher, markerArray, fire_topic, people_topic, fireRange, peopleRange, wallStart
        if not wallStart: return
        publisher.publish(markerArray)
        T2 = get_telemetry(frame_id = "aruco_map")
        x1, y1, z1 = T2.x, T2.y, T2.z

        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        if checkForObject(img, fireRange, (255, 0, 0)):
            addFire(x1, y1)

        if checkForObject(img, peopleRange, (0, 0, 255)):
            print(x1, y1)

        fire_topic.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

    # Топика для получения кадров с камеры RPI с частототой 10 Гц
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

    # Стандартные функции
    # rospy.init_node('flight')
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
    set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
    set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
    land = rospy.ServiceProxy('land', Trigger)

    # Топик для получения лазерного дальномера
    rospy.Subscriber('rangefinder/range', Range, range_callback)

    # Движение к точке
    navigate(z=1, speed=1, frame_id='body', auto_arm=True)
    rospy.sleep(5)

    T = getTelem()

    x0, y0, z0 = T.x, T.y, T.z

    #navigate_wait(x=x0, y=0, z=1, frame_id="aruco_map")
    #navigate_wait(x=0, y=0, z=1, frame_id="aruco_map")
    navigate_wait(x=0, y=y0, z=1, frame_id="aruco_map")

    navigate_wait(x=0, y=4 * 1.005, z=1, frame_id="aruco_map")
    navigate_wait(x=0, y=4 * 1.005, z=1, frame_id="aruco_map", yaw=math.pi)
    rospy.sleep(5)
    wallStart = True
    navigate_wait(x=7 * 1.005, y=4 * 1.005, z=1, yaw=math.pi, speed=0.2, frame_id="aruco_map")

    land()

    # Вывод автоматического отчёта
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
    for i in range(1, len(data)):
        last = data[i - 1]
        dista, x, y = data[i]
        c += 1
        print("Wall", str(c) + ":", get_distance(x, y, 0, last[1], last[2], 0))

        c += 1
        print("Wall", str(c) + ":", abs(last[0] - dista)
'''
