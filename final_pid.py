from time import sleep
from gpiozero import Motor, LineSensor
motor1 = Motor(forward=9, backward=10)
motor2 = Motor(forward=8, backward=7)

LineSensor0 = LineSensor(19)
LineSensor1 = LineSensor(16)
LineSensor2 = LineSensor(20)  # change
LineSensor3 = LineSensor(21)  # change

sensor[4] = [0, 0, 0, 0]
speed = 50
Kp = 25
Ki = 0
Kd = 15
e = 0
P = 0
I = 0
D = 0
PID_value = 0
previous_e = 0
previous_I = 0
flag = 0


def read_sensor_values:
    sensor[0] = int(LineSensor0.value)
    sensor[1] = int(LineSensor1.value)
    sensor[2] = int(LineSensor2.value)
    sensor[3] = int(LineSensor3.value)
    if (sensor[0] == 1) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 0):
        e = 3
    elif ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 0) and (sensor[3] == 0)):
        e = 2
    elif ((sensor[0] == 0) and (sensor[1] == 1) and (sensor[2] == 0) and (sensor[3] == 0)):
        e = 1
    elif ((sensor[0] == 0) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 0)):
        e = 0
    elif ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 1) and (sensor[3] == 0)):
        e = -1
    elif ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 1) and (sensor[3] == 1)):
        e = -2
    elif ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 1)):
        e = -3
    # Turn robot left side
    elif ((sensor[0] == 1) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 0)):
        e = 4
    # Turn robot right side
    elif ((sensor[0] == 0) and (sensor[1] == 1) and (sensor[2] == 1) and (sensor[3] == 1)):
        e = -4
    elif ((sensor[0] == 0) and (sensor[1] == 0) and (sensor[2] == 0) and (sensor[3] == 0)):  # Make U turn
        e = 5


def calculate_pid:
    P = e
    I = I + previous_I
    D = e - previous_e

    PID_value = (Kp * P) + (Ki * I) + (Kd * D)
    previous_I = I
    previous_e = e


def motor_control:
    # Calculating the effective motor speed
    left_motor_speed = initial_motor_speed - PID_value
    right_motor_speed = initial_motor_speed + PID_value
    # to prevent values higher than 100
    if left_motor_speed > 100:
        left_motor_speed = 100
    if right_motor_speed > 100:
        right_motor_speed = 100
    #to prevent negative values
    if left_motor_speed < 0:
        left_motor_speed = 0
    if right_motor_speed < 0:
        right_motor_speed = 0

    left_motor_speed = left_motor_speed/100
    right_motor_speed = right_motor_speed/100
    motor1.forward(left_motor_speed)
    motor2.forward(right_motor_speed)


while True:
    read_sensor_values()
    print(e)
    if e == 6:
        e = 0
 	else:
 		calculate_pid()
        motor_control()
        
                    
        
    










