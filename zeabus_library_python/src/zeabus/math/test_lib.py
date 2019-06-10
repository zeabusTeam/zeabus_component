import math
from quaternion import Quaternion
q1 = Quaternion()
q2 = Quaternion()
q3 = Quaternion()
q5 = Quaternion()
q1.set_quaternion((1,-2,3,4))
q2.set_quaternion((-5,6,7,8))
q3 = q1*q2
q3.print_quaternion()
q3.inverse()
q3.print_quaternion()
q4 = q1-q1
q4.print_quaternion()
q1.set_quaternion((12,15,14,13))
q1.nomalize()
q1.print_quaternion()
q5 = q1.rotation(math.pi,[ 0.37598996,  0.32275639,  0.31308907])
q5.print_quaternion()
q1.print_quaternion()
q1.set_euler(1,2,3)
