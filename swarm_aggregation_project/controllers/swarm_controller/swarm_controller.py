from controller import Supervisor
import math

TIME_STEP = 32
ROBOT_RADIUS = 0.145
WORLD_LIMIT = 3.72
COMMUNICATION_RANGE = 2.3
OBSTACLE_SENSOR_RANGE = 0.85
ARRIVAL_MARGIN = 0.10
MAX_LINEAR_SPEED = 0.18
MAX_ANGULAR_SPEED = 1.35

class SwarmRobot:
    def __init__(self):
        self.robot = Supervisor()
        self.self_node = self.robot.getSelf()
        self.name = self.robot.getName()
        self.translation_field = self.self_node.getField("translation")
        self.rotation_field = self.self_node.getField("rotation")
        self.custom_data_field = self.self_node.getField("customData")
        self.gps = self.robot.getDevice("gps")
        self.imu = self.robot.getDevice("inertial unit")
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        if self.gps:
            self.gps.enable(TIME_STEP)
        if self.imu:
            self.imu.enable(TIME_STEP)
        if self.receiver:
            self.receiver.enable(TIME_STEP)
        self.scenario = "easy"
        self.reset_id = "0"
        self.target = (0.0, 0.0)
        self.target_radius = 0.90
        self.obstacles = []
        self.neighbors = {}
        self.last_payload = ""
        self.arrived = False
        self.last_time = self.robot.getTime()
        self.load_custom_data(True)

    def parse_payload(self, payload):
        data = {}
        for item in payload.split(";"):
            if "=" in item:
                key, value = item.split("=", 1)
                data[key.strip()] = value.strip()
        return data

    def load_custom_data(self, force=False):
        payload = self.custom_data_field.getSFString()
        if not force and payload == self.last_payload:
            return
        old_reset = self.reset_id
        self.last_payload = payload
        data = self.parse_payload(payload)
        self.scenario = data.get("scenario", self.scenario)
        self.reset_id = data.get("reset", self.reset_id)
        if "target" in data:
            parts = data["target"].split(",")
            if len(parts) >= 2:
                self.target = (float(parts[0]), float(parts[1]))
        if "radius" in data:
            self.target_radius = float(data["radius"])
        self.obstacles = []
        if data.get("obstacles", ""):
            for item in data["obstacles"].split("|"):
                parts = item.split(",")
                if len(parts) >= 3:
                    self.obstacles.append((float(parts[0]), float(parts[1]), float(parts[2])))
        if force or old_reset != self.reset_id:
            self.neighbors.clear()
            self.arrived = False

    def position(self):
        p = self.translation_field.getSFVec3f()
        return p[0], p[1]

    def yaw(self):
        r = self.rotation_field.getSFRotation()
        return r[3] if r[2] >= 0 else -r[3]

    def normalize(self, x, y):
        d = math.hypot(x, y)
        if d < 1e-9:
            return 0.0, 0.0, 0.0
        return x / d, y / d, d

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def angle_error(self, target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def receive_neighbors(self, now):
        if not self.receiver:
            return
        while self.receiver.getQueueLength() > 0:
            try:
                msg = self.receiver.getString()
                parts = msg.split(",")
                if len(parts) >= 6:
                    name = parts[0]
                    scenario = parts[4]
                    reset = parts[5]
                    if name != self.name and scenario == self.scenario and reset == self.reset_id:
                        self.neighbors[name] = (float(parts[1]), float(parts[2]), now)
            except Exception:
                pass
            self.receiver.nextPacket()
        expired = [name for name, item in self.neighbors.items() if now - item[2] > 0.9]
        for name in expired:
            del self.neighbors[name]

    def send_state(self, x, y):
        if self.emitter:
            self.emitter.send(f"{self.name},{x:.4f},{y:.4f},{int(self.arrived)},{self.scenario},{self.reset_id}".encode("utf-8"))

    def target_force(self, x, y):
        dx = self.target[0] - x
        dy = self.target[1] - y
        ux, uy, dist = self.normalize(dx, dy)
        if dist <= self.target_radius - ARRIVAL_MARGIN:
            return 0.0, 0.0, dist
        strength = self.clamp(0.16 + dist * 0.10, 0.12, 0.55)
        return ux * strength, uy * strength, dist

    def neighbor_force(self, x, y):
        fx = 0.0
        fy = 0.0
        for nx, ny, _ in self.neighbors.values():
            dx = x - nx
            dy = y - ny
            ux, uy, d = self.normalize(dx, dy)
            if d <= 0.001 or d > COMMUNICATION_RANGE:
                continue
            if d < 0.34:
                s = (0.34 - d) * 2.0
                fx += ux * s
                fy += uy * s
            elif d < 1.55:
                fx += (nx - x) * 0.012
                fy += (ny - y) * 0.012
        return fx, fy

    def obstacle_force(self, x, y):
        fx = 0.0
        fy = 0.0
        for ox, oy, radius in self.obstacles:
            dx = x - ox
            dy = y - oy
            ux, uy, d = self.normalize(dx, dy)
            safe = radius + ROBOT_RADIUS + 0.34
            if d < safe + OBSTACLE_SENSOR_RANGE:
                if d < 0.001:
                    ux, uy = 1.0, 0.0
                    d = 0.001
                s = (safe + OBSTACLE_SENSOR_RANGE - d) / OBSTACLE_SENSOR_RANGE
                fx += ux * s * 0.95
                fy += uy * s * 0.95
        return fx, fy

    def boundary_force(self, x, y):
        fx = 0.0
        fy = 0.0
        margin = 0.55
        if x > WORLD_LIMIT - margin:
            fx -= (x - (WORLD_LIMIT - margin)) * 1.6
        if x < -WORLD_LIMIT + margin:
            fx += ((-WORLD_LIMIT + margin) - x) * 1.6
        if y > WORLD_LIMIT - margin:
            fy -= (y - (WORLD_LIMIT - margin)) * 1.6
        if y < -WORLD_LIMIT + margin:
            fy += ((-WORLD_LIMIT + margin) - y) * 1.6
        return fx, fy

    def move(self, fx, fy, dt):
        x, y = self.position()
        yaw = self.yaw()
        _, _, mag = self.normalize(fx, fy)
        if mag < 0.02:
            return
        desired_heading = math.atan2(fy, fx)
        error = self.angle_error(desired_heading, yaw)
        angular = self.clamp(error * 1.7, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        linear = self.clamp(mag * 0.23, 0.0, MAX_LINEAR_SPEED) * max(0.0, math.cos(error))
        if abs(error) > 1.65:
            linear = 0.0
        yaw = yaw + angular * dt
        nx = x + linear * math.cos(yaw) * dt
        ny = y + linear * math.sin(yaw) * dt
        nx = self.clamp(nx, -WORLD_LIMIT, WORLD_LIMIT)
        ny = self.clamp(ny, -WORLD_LIMIT, WORLD_LIMIT)
        for ox, oy, radius in self.obstacles:
            dx = nx - ox
            dy = ny - oy
            ux, uy, d = self.normalize(dx, dy)
            min_d = radius + ROBOT_RADIUS + 0.04
            if d < min_d:
                if d < 0.001:
                    ux, uy = math.cos(yaw), math.sin(yaw)
                nx = ox + ux * min_d
                ny = oy + uy * min_d
        self.translation_field.setSFVec3f([nx, ny, 0.09])
        self.rotation_field.setSFRotation([0.0, 0.0, 1.0, yaw])

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            now = self.robot.getTime()
            dt = max(0.001, min(0.08, now - self.last_time))
            self.last_time = now
            self.load_custom_data(False)
            x, y = self.position()
            self.receive_neighbors(now)
            _, _, dist = self.target_force(x, y)
            if dist <= self.target_radius - ARRIVAL_MARGIN:
                self.arrived = True
            self.send_state(x, y)
            if self.arrived:
                self.translation_field.setSFVec3f([x, y, 0.09])
                continue
            tx, ty, _ = self.target_force(x, y)
            nx, ny = self.neighbor_force(x, y)
            ox, oy = self.obstacle_force(x, y)
            bx, by = self.boundary_force(x, y)
            self.move(tx + nx + ox + bx, ty + ny + oy + by, dt)

if __name__ == "__main__":
    SwarmRobot().run()