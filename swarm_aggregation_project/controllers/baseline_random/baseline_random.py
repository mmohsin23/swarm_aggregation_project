from controller import Robot
import random

TIME_STEP = 64
robot = Robot()
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
timer = 0
left_speed = 0.0
right_speed = 0.0
while robot.step(TIME_STEP) != -1:
    if timer <= 0:
        left_speed = random.uniform(-2.5, 2.5)
        right_speed = random.uniform(-2.5, 2.5)
        timer = 55
    timer -= 1
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
