from time import sleep
from gpiozero import Motor, LineSensor

motor1 = Motor(forward=19, backward=26)
motor2 = Motor(forward=13, backward=15)

LineSensor3 = LineSensor(22)
LineSensor2 = LineSensor(27)
LineSensor1 = LineSensor(17)
LineSensor0 = LineSensor(4)

initial_motor_speed = 60
Kp = 30
Ki = 0
Kd = 0
e = 0
P = 0
I = 0
D = 0
PID_value = 0
previous_e = 0
previous_I = 0
flag = 0


def read_sensor_values():
    sensor0 = int(LineSensor0.value)
    sensor1 = int(LineSensor1.value)
    sensor2 = int(LineSensor2.value)
    sensor3 = int(LineSensor3.value)
    if ((sensor0 == 1) and (sensor1 == 0) and (sensor2 == 0) and (sensor3 == 0)):
        e = 3
        return e
    elif ((sensor0 == 1) and (sensor1 == 1) and (sensor2 == 0) and (sensor3 == 0)):
        e = 2
        return e
    elif ((sensor0 == 0) and (sensor1 == 1) and (sensor2 == 0) and (sensor3 == 0)):
        e = 1
        return e
    elif ((sensor0 == 0) and (sensor1 == 1) and (sensor2 == 1) and (sensor3 == 0)):
        e = 0
        return e
    elif ((sensor0 == 0) and (sensor1 == 0) and (sensor2 == 1) and (sensor3 == 0)):
        e = -1
        return e
    elif ((sensor0 == 0) and (sensor1 == 0) and (sensor2 == 1) and (sensor3 == 1)):
        e = -2
        return e
    elif ((sensor0 == 0) and (sensor1 == 0) and (sensor2 == 0) and (sensor3 == 1)):
        e = -3
        return e
    # Turn robot left side
    elif ((sensor0 == 1) and (sensor1 == 1) and (sensor2 == 1) and (sensor3 == 0)):
        e = 4
        return e
    # Turn robot right side
    elif ((sensor0 == 0) and (sensor1 == 1) and (sensor2 == 1) and (sensor3 == 1)):
        e = -4
        return e
    elif ((sensor0 == 0) and (sensor1 == 0) and (sensor2 == 0) and (sensor3 == 0)):  # Make U turn
        e = 5
        return e
    else:
        e = 6
        return e


def calculate_pid():
    global e, P, I, D, previous_I, previous_e, PID_value, Kp, Ki, Kd
    P = e
    I = I + previous_I
    D = e - previous_e

    PID_value = (Kp * P) + (Ki * I) + (Kd * D)
    previous_I = I
    previous_e = e


def motor_control():
    global initial_motor_speed, PID_value
    # Calculating the effective motor speed
    left_motor_speed = initial_motor_speed - PID_value
    right_motor_speed = initial_motor_speed + PID_value
    # to prevent negative values
    if left_motor_speed < 0:
        left_motor_speed = left_motor_speed * -1
    if right_motor_speed < 0:
        right_motor_speed = right_motor_speed * -1
    # to prevent values higher than 100
    if left_motor_speed > 100:
        left_motor_speed = 100
    if right_motor_speed > 100:
        right_motor_speed = 100

    left_motor_speed = left_motor_speed / 100
    right_motor_speed = right_motor_speed / 100
    motor1.forward(left_motor_speed)
    motor2.forward(right_motor_speed)


while True:
    e = read_sensor_values()
    print(e)
    if ((e == 6) or (e == 4) or (e == -4) or (e == 5)):
        e = 0
    else:
        calculate_pid()
        motor_control()








