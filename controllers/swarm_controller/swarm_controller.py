from controller import Robot, Emitter, Receiver
import math

TIME_STEP = 64
TARGET = (0.0, 0.0)
STOP_DIST = 0.3

class SwarmRobot:
    def __init__(self):
        self.robot = Robot()
        self.name = self.robot.getName()
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        self.state = "SEARCH"
        self.search_dir = 1
        self.last_dist = None
        self.search_start = None
        self.move_start = None
        self.print_timer = 0

    def get_position(self):
        p = self.gps.getValues()
        return (p[0], p[2])

    def distance_to_target(self, pos):
        return math.hypot(pos[0] - TARGET[0], pos[1] - TARGET[1])

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            pos = self.get_position()
            dist = self.distance_to_target(pos)
            t = self.robot.getTime()

            if t - self.print_timer > 2.0:
                self.print_timer = t
                print(f"{self.name}: t={t:.1f} pos=({pos[0]:.2f},{pos[1]:.2f}) dist={dist:.2f} state={self.state}")

            if dist < STOP_DIST:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                continue

            if self.state == "SEARCH":
                rot_speed = 1.2
                self.left_motor.setVelocity(-self.search_dir * rot_speed)
                self.right_motor.setVelocity(self.search_dir * rot_speed)

                if self.search_start is None:
                    self.search_start = t
                    self.last_dist = dist
                elif t - self.search_start > 1.2:
                    new_dist = self.distance_to_target(self.get_position())
                    if new_dist < self.last_dist - 0.01:
                        self.state = "MOVE"
                        self.move_start = t
                        self.search_start = None
                        print(f"{self.name}: direction found")
                    else:
                        self.search_dir *= -1
                        self.search_start = t
                        self.last_dist = new_dist
                        print(f"{self.name}: wrong direction, reversing")

            elif self.state == "MOVE":
                speed = 1.2
                self.left_motor.setVelocity(speed)
                self.right_motor.setVelocity(speed)

                if self.move_start is None:
                    self.move_start = t
                    self.last_dist = dist
                elif t - self.move_start > 2.0:
                    new_dist = self.distance_to_target(self.get_position())
                    if new_dist > self.last_dist + 0.05:
                        self.state = "SEARCH"
                        self.search_dir *= -1
                        self.search_start = None
                        self.move_start = None
                        print(f"{self.name}: moving away, re-searching")
                    else:
                        self.move_start = t
                        self.last_dist = new_dist

            x, z = pos
            self.emitter.send(f"{self.name},{x},{z}".encode())
            while self.receiver.getQueueLength() > 0:
                self.receiver.nextPacket()

if __name__ == "__main__":
    robot = SwarmRobot()
    robot.run()