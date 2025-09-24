#подключение библиотек
import serial
import io
import time
import os
import OZK
##Установка параметров порта##
if os.name == 'nt':
 import msvcrt
 def getch():
    return msvcrt.getch().decode()
else:
 import sys, tty, termios
 fd = sys.stdin.fileno()
 old_settings = termios.tcgetattr(fd)
 def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
from dynamixel_sdk import *
# создаём объект класса RobotArm из OZK и объявляем переменные адресов регистров соответственно состояния, позиции, предыдущей позиции и скорости движения
robot = OZK.RoboticArm()
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30 
ADDR_MX_PRESENT_POSITION = 36 
ADDR_MX_GOAL_SPEED = 32 
# Задаём версию протокола 
PROTOCOL_VERSION = 1.0 
# Устанавливаем ID, скорость обмена данными и имя DXL устройства в сети
DXL_ID = [1, 2, 3, 4] 
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyS2'
# Устанавливаем «макросы» на вкл/выкл питания на моторе
TORQUE_ENABLE = 1 
TORQUE_DISABLE = 0 
# Создаём список предыдущих позиций, темповый список текущих позиций и желаемую скорость движения
dxl_goal_position_tmp = []
dxl_goal_speed = 50
dxl_present_position = [-1, -1, -1, -1]
# Для связи Open CM c nanopi получаем дескриптор порта, устанавливаем скорость обмена данными, создаём portHandler и packetHandler для управления портом, приём-передачи данных
ocm = serial.Serial(port='/dev/ttyS1', baudrate=115200)
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
#Открываем порт для обмена-передачи данных
if portHandler.openPort():
 print("Succeeded to open the port")
else:
 print("Failed to open the port")
 print("Press any key to terminate...")
 getch()#очищение холста и ожидание нажатия кнопки
 quit()#выход
# Устанавливаем скорость обмена данными
if portHandler.setBaudRate(BAUDRATE):
 print("Succeeded to change the baudrate")
else:
 print("Failed to change the baudrate")
 print("Press any key to terminate...")
 getch()#очищение холста и ожидание нажатия кнопки
 quit()#выход


#Устанавливаем скорость движения звеньев в соответствии с dxl_goal_speed
if ([packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_SPEED, dxl_goal_speed) for dxl_id in DXL_ID]):
  print("Speed change successful")
else:
  print("Speed change successful")

# Включение моторов двигателей
if not ([packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE) for dxl_id in DXL_ID]):
  print("ERROR")
else:
  print("Dynamixel has been successfully connected")

####Установка параметров###
#####Рабочий режим####
while 1:
 #Вводим координаты целевой точки по XYZ в миллиметрах, и также задаём режим работы помпы
 print("Press any key to continue! (or press ESC to quit!)")
 if getch() == chr(0x1b):# нажали esc
    break
 dxl_goal_position = []
 print("Write Goal X Position:\n")
 X = int(input())
 print("Write Goal Y Position:\n")
 Y = int(input())
 print("Write Goal Z Position:\n")
 Z = int(input())
 print("Write pump state (1 - ON, 2 - OFF):\n")
 pump_state = input()
 #запись состояния пневмосистемы на Open CM
 ocm.write(pump_state.encode())

 #Решаем обратную задачу кинематики, заполняем список соответствующими переменными
 dxl_goal_position_tmp = robot.InversProblem(X, Y, Z)
 for i in dxl_goal_position_tmp:
    dxl_goal_position.append(int(i))
 print(dxl_goal_position)

 #Отправляем команды на моторы
 for i in range(len(DXL_ID)):
  #запись позиций сервоприводов в регистр ADDR_MX_GOAL_POSITION
  packetHandler.write2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_GOAL_POSITION, dxl_goal_position[i])
 while 1:
    for i in range(len(DXL_ID)):
        #чтение текущей позиции сервоприводов из регистра ADDR_MX_PRESENT_POSITION
        dxl_present_position[i] = packetHandler.read2ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION)
        print(dxl_present_position)
        print("[ID:%03d] GoalPos:%03d PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position[i][0]))
    if i == len(DXL_ID)-1:
        break
###Рабочий режим###
###Отключение###
#Отключаем моторов
[packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE) for dxl_id in DXL_ID]
ocm.write(b"0")
#Закрываем порт
portHandler.closePort()
###Отключение###