from math import *#импорт переменных из math
import numpy as np #подключение библиотеки numpy
class RoboticArm:
 def __init__(self):
 #Объявляем длины звеньев манипулятора и разность между осями в порядке «От земли до захвата»
    self.__l1 = 124
    self.__l2 = 20
    self.__l3 = 150
    self.__l4 = 150
    self.__l5 = 40
    self.__l6 = 155
    self.__l7 = 2.25
 def InversProblem(self,X,Y,Z,pitch = 0):
 #Присваивание соответствующих величин для удобства работы с ними
    l1 = self.__l1
    l2 = self.__l2
    l3 = self.__l3
    l4 = self.__l4
    l5 = self.__l5
    l6 = self.__l6
    l7 = self.__l7
    try:
    #Вычисление всех углов и длин, а также преобразование координат в соответствии с методом решения ОЗК
        alpha_temp = atan2(Y,X)
        tetta = asin(l7/sqrt(X**2+Y**2))
        alpha1 = alpha_temp+tetta
        l = sqrt(X**2+Y**2-l7**2)
        X = l*cos(alpha1)
        Y = l*sin(alpha1)
        z = Z+l6-l1
        x = X/cos(alpha1)
        x = x - l2 - l5 
        d = sqrt(x*x+z*z)
        gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
        beta = gamma + atan(z/x)
        alpha2 = pi/2 - beta
        gamma1 = acos((l3*l3+l4*l4-d*d)/(2*l3*l4))
        alpha3 = gamma1 - alpha2
        s1 = alpha2
        s2 = alpha3
 
        #Приведение полученных значений к соответствующим управляющим сигналам
        tick1 = alpha1*(180/pi)*11.378+2048 
        tick2 = 2048-alpha2*(180/pi)*11.378
        tick3 = 2048+(pi/2 - alpha3)*(180/pi)*11.378
        tick4 = pitch*(180/pi)*11.378
        q = (tick1 ,tick2 ,tick3 ,512)
        print(q)
        return q
    except Exception as e:
    #В случае ошибки при вычислении ОЗК выведет сообщение об ошибке
        print (e)
        return