import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Инициализация ROS узла
rospy.init_node('snake_mission')

# Создание прокси для сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)

# Инициализация CvBridge для работы с изображениями
bridge = CvBridge()

# Глобальные переменные
color = 'error'
objects_data = {}

# Функция для распознавания цвета объекта в HSV
def color_callback(data):
    global color
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    # Обрезка изображения для анализа центральной области
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[119:120, 159:160]
    telemetry = get_telemetry(frame_id='aruco_map')
    # Диапазоны HSV для цветов
    red_low_value = (0, 150, 200)
    red_high_value = (10, 255, 255)
    green_low_value = (35, 100, 100)
    green_high_value = (85, 255, 255)
    blue_low_value = (100, 150, 100)
    blue_high_value = (130, 255, 255)
    
    # Проверка на красный цвет
    red_final = cv2.inRange(img_hsv, red_low_value, red_high_value)
    if red_final[0][0] == 255:
        color = 'red'
        print(telemetry.x, telemetry.y, telemetry.z)
        print(telemetry.z)
    # Проверка на зелёный цвет
    green_final = cv2.inRange(img_hsv, green_low_value, green_high_value)
    if green_final:
        color = 'green'
    # Проверка на синий цвет
    blue_final = cv2.inRange(img_hsv, blue_low_value, blue_high_value)
    if blue_final[0][0] == 255:
        color = 'blue'
    else:
        color = 'error'

# Подписка на топик с изображением с камеры
image_sub = rospy.Subscriber("main_camera/image_raw_throttled", Image, color_callback)

# Функция для полёта по змейке
def snake_flight():
    # Взлет
    navigate(x=0, y=0, z=2, frame_id='body', auto_arm=True)
    rospy.sleep(5)
    
    # Параметры змейки
    x_start, x_end = 1, 6  # Диапазон по X
    y_start, y_end = 1, 4  # Диапазон по Y
    z = 2  # Высота полёта
    step = 1  # Шаг перемещения
    
    # Полёт по змейке
    for y in range(y_start, y_end + 1, step):
        for x in range(x_start, x_end + 1, step):
            # Перемещение к точке
            navigate(x=x, y=y, z=z, frame_id='aruco_map')
            rospy.sleep(3)  # Ожидание для стабилизации
            
            # Получение текущих координат
            telem = get_telemetry(frame_id='aruco_map')
            x_pos, y_pos = telem.x, telem.y
            
            # Сохранение данных о цвете и координатах
            if color != 'error':
                object_id = len(objects_data) + 1
                objects_data[object_id] = {'color': color, 'x': x_pos, 'y': y_pos}
                print(f'Object {object_id}: {color} {x_pos} {y_pos}')
            
            # Включение светодиодной индикации
            set_effect(effect='blink', r=(255 if color == 'red' else 0),
                        g=(255 if color == 'green' else 0), b=(255 if color == 'blue' else 0))
        
        # Обратный ход для змейки
        x_start, x_end = x_end, x_start
        step = -step

    # Посадка на зарядную станцию
    land()

# Запуск миссии
snake_flight()

# Генерация отчёта
with open('object_report.txt', 'w') as f:
    for obj_id, data in objects_data.items():
        f.write(f'Object {obj_id}: {data["color"]} {data["x"]} {data["y"]}\n')

print("Миссия завершена. Отчёт сгенерирован.")