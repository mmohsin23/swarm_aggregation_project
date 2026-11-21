from controller import Robot
import random

TIME_STEP = 64
robot = Robot()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

timer = 0
change_interval = 60  # steps

while robot.step(TIME_STEP) != -1:
    if timer <= 0:
        left_spd = random.uniform(-3, 3)
        right_spd = random.uniform(-3, 3)
        timer = change_interval
    else:
        timer -= 1
    left_motor.setVelocity(left_spd)
    right_motor.setVelocity(right_spd)