from controller import Supervisor
from pathlib import Path
from datetime import datetime
import csv
import math

TIME_STEP = 32
ROBOT_COUNT = 6
COLLISION_DISTANCE = 0.30
ACHIEVED_HOLD_TIME = 1.0

SCENARIOS = {
    "easy": {
        "radius": 0.90,
        "robots": [
            (-2.45, -1.75, 0.30),
            (-2.40, 1.75, -0.30),
            (2.45, -1.75, 2.80),
            (2.40, 1.75, -2.80),
            (-0.75, -2.75, 1.35),
            (0.75, 2.75, -1.35)
        ],
        "obstacles": []
    },
    "medium": {
        "radius": 0.90,
        "robots": [
            (-3.05, -2.70, 0.35),
            (-3.05, 2.70, -0.35),
            (3.05, -2.70, 2.65),
            (3.05, 2.70, -2.65),
            (-0.85, -3.15, 1.35),
            (0.85, 3.15, -1.35)
        ],
        "obstacles": [
            (-1.85, 1.15, 0.28),
            (1.85, -1.15, 0.28),
            (0.00, 2.35, 0.25)
        ]
    },
    "hard": {
        "radius": 0.95,
        "robots": [
            (-3.25, -3.05, 0.45),
            (-3.25, 3.05, -0.45),
            (3.25, -3.05, 2.55),
            (3.25, 3.05, -2.55),
            (-1.25, -3.30, 1.25),
            (1.25, 3.30, -1.25)
        ],
        "obstacles": [
            (-2.20, -0.95, 0.30),
            (2.20, 0.95, 0.30),
            (-1.15, 2.25, 0.27),
            (1.15, -2.25, 0.27),
            (0.00, 3.05, 0.24)
        ]
    }
}

class ScenarioSupervisor:
    def __init__(self):
        self.sup = Supervisor()
        self.time_step = int(self.sup.getBasicTimeStep()) or TIME_STEP
        self.project_root = Path(__file__).resolve().parents[2]
        self.metrics_dir = self.project_root / "metrics"
        self.runtime_dir = self.project_root / "runtime"
        self.metrics_dir.mkdir(exist_ok=True)
        self.runtime_dir.mkdir(exist_ok=True)
        self.state_file = self.runtime_dir / "selected_scenario.txt"
        self.robots = [self.sup.getFromDef(f"robot{i}") for i in range(1, ROBOT_COUNT + 1)]
        self.obstacle_nodes = [self.sup.getFromDef(f"obstacle{i}") for i in range(1, 6)]
        self.obstacle_geometries = [self.sup.getFromDef(f"obstacle{i}_cylinder") for i in range(1, 6)]
        self.target_cylinder = self.sup.getFromDef("target_cylinder")
        self.target_node = self.sup.getFromDef("target_region")
        self.keyboard = self.sup.getKeyboard()
        self.keyboard.enable(self.time_step)
        self.pending_scenario = None
        self.root = None
        self.labels = {}
        self.create_gui()
        self.scenario = self.read_selected_scenario()
        self.reset_id = "0"
        self.start_time = 0.0
        self.last_sample_time = -10.0
        self.samples = []
        self.path_lengths = [0.0 for _ in range(ROBOT_COUNT)]
        self.last_positions = [None for _ in range(ROBOT_COUNT)]
        self.active_collision_pairs = set()
        self.collision_count = 0
        self.achieved_since = None
        self.metrics_written = False
        self.configure(self.scenario)

    def create_gui(self):
        try:
            import tkinter as tk
            self.tk = tk
            self.root = tk.Tk()
            self.root.title("Swarm Scenario Control")
            self.root.geometry("330x220+80+80")
            self.root.attributes("-topmost", True)
            frame = tk.Frame(self.root, padx=12, pady=12)
            frame.pack(fill="both", expand=True)
            tk.Label(frame, text="Swarm Aggregation", font=("Arial", 16, "bold")).pack(anchor="w", pady=(0, 10))
            row = tk.Frame(frame)
            row.pack(fill="x")
            for scenario in ("easy", "medium", "hard"):
                tk.Button(row, text=scenario.upper(), width=9, height=2, command=lambda s=scenario: self.request_scenario(s)).pack(side="left", padx=3)
            status = tk.Frame(frame)
            status.pack(fill="x", pady=(12, 0))
            for key in ("Scenario", "Time", "Inside", "Average", "Collisions", "Metrics"):
                line = tk.Frame(status)
                line.pack(fill="x", anchor="w")
                tk.Label(line, text=key + ":", width=10, anchor="w").pack(side="left")
                value = tk.Label(line, text="-", anchor="w")
                value.pack(side="left")
                self.labels[key] = value
            self.root.protocol("WM_DELETE_WINDOW", self.root.withdraw)
            self.root.update()
        except Exception as e:
            print("Scenario buttons unavailable, use keyboard 1/2/3:", e)
            self.root = None

    def request_scenario(self, scenario):
        if scenario in SCENARIOS:
            self.pending_scenario = scenario

    def update_gui(self, inside=None, avg_distance=None, elapsed=None):
        if not self.root:
            return
        try:
            if inside is None:
                inside = 0
            if avg_distance is None:
                avg_distance = 0.0
            if elapsed is None:
                elapsed = 0.0
            self.labels["Scenario"].config(text=self.scenario)
            self.labels["Time"].config(text=f"{elapsed:.1f} s")
            self.labels["Inside"].config(text=f"{inside}/{ROBOT_COUNT}")
            self.labels["Average"].config(text=f"{avg_distance:.3f} m")
            self.labels["Collisions"].config(text=str(self.collision_count))
            self.labels["Metrics"].config(text="generated" if self.metrics_written else "running")
            self.root.update_idletasks()
            self.root.update()
        except Exception:
            self.root = None

    def read_selected_scenario(self):
        if self.state_file.exists():
            value = self.state_file.read_text().strip().lower()
            if value in SCENARIOS:
                return value
        self.state_file.write_text("easy\n")
        return "easy"

    def payload(self, scenario):
        conf = SCENARIOS[scenario]
        obstacles = "|".join(f"{x:.3f},{y:.3f},{r:.3f}" for x, y, r in conf["obstacles"])
        return f"scenario={scenario};reset={self.reset_id};target=0.000,0.000;radius={conf['radius']:.3f};obstacles={obstacles}"

    def configure(self, scenario):
        if scenario not in SCENARIOS:
            scenario = "easy"
        self.scenario = scenario
        self.state_file.write_text(scenario + "\n")
        self.reset_id = datetime.now().strftime("%H%M%S%f")
        conf = SCENARIOS[scenario]
        if self.target_node:
            self.target_node.getField("translation").setSFVec3f([0.0, 0.0, 0.016])
        if self.target_cylinder:
            self.target_cylinder.getField("radius").setSFFloat(conf["radius"])
        data = self.payload(scenario)
        for i, node in enumerate(self.robots):
            if not node:
                continue
            x, y, yaw = conf["robots"][i]
            node.getField("translation").setSFVec3f([x, y, 0.09])
            node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, yaw])
            node.getField("customData").setSFString(data)
            node.resetPhysics()
        for i, node in enumerate(self.obstacle_nodes):
            if not node:
                continue
            if i < len(conf["obstacles"]):
                x, y, r = conf["obstacles"][i]
                node.getField("translation").setSFVec3f([x, y, 0.15])
                if i < len(self.obstacle_geometries) and self.obstacle_geometries[i]:
                    self.obstacle_geometries[i].getField("radius").setSFFloat(r)
            else:
                node.getField("translation").setSFVec3f([0.0, 0.0, -1.2])
            node.resetPhysics()
        self.sup.simulationResetPhysics()
        self.reset_metrics()
        print("Scenario selected:", scenario)

    def reset_metrics(self):
        self.start_time = self.sup.getTime()
        self.last_sample_time = -10.0
        self.samples = []
        self.path_lengths = [0.0 for _ in range(ROBOT_COUNT)]
        self.last_positions = [None for _ in range(ROBOT_COUNT)]
        self.active_collision_pairs = set()
        self.collision_count = 0
        self.achieved_since = None
        self.metrics_written = False

    def positions(self):
        data = []
        for node in self.robots:
            if node:
                p = node.getPosition()
                data.append((p[0], p[1]))
        return data

    def update_paths(self, positions):
        for i, p in enumerate(positions):
            last = self.last_positions[i]
            if last is not None:
                step = math.hypot(p[0] - last[0], p[1] - last[1])
                if step < 0.20:
                    self.path_lengths[i] += step
            self.last_positions[i] = p

    def update_collisions(self, positions):
        current = set()
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                d = math.hypot(positions[i][0] - positions[j][0], positions[i][1] - positions[j][1])
                if d < COLLISION_DISTANCE:
                    pair = (i + 1, j + 1)
                    current.add(pair)
                    if pair not in self.active_collision_pairs:
                        self.collision_count += 1
        self.active_collision_pairs = current

    def collect_metrics(self):
        now = self.sup.getTime()
        elapsed = now - self.start_time
        positions = self.positions()
        if not positions:
            return
        radius = SCENARIOS[self.scenario]["radius"]
        distances = [math.hypot(x, y) for x, y in positions]
        inside = sum(1 for d in distances if d <= radius)
        avg_distance = sum(distances) / len(distances)
        self.update_paths(positions)
        self.update_collisions(positions)
        if now - self.last_sample_time >= 0.25:
            row = {
                "time": round(elapsed, 3),
                "scenario": self.scenario,
                "inside_count": inside,
                "inside_percentage": round(inside * 100.0 / ROBOT_COUNT, 2),
                "avg_distance_to_target": round(avg_distance, 4),
                "collision_count": self.collision_count,
                "total_path_length": round(sum(self.path_lengths), 4)
            }
            for i, d in enumerate(distances, start=1):
                row[f"robot{i}_distance_to_target"] = round(d, 4)
                row[f"robot{i}_path_length"] = round(self.path_lengths[i - 1], 4)
            self.samples.append(row)
            self.last_sample_time = now
        if inside == ROBOT_COUNT:
            if self.achieved_since is None:
                self.achieved_since = now
            if now - self.achieved_since >= ACHIEVED_HOLD_TIME and not self.metrics_written:
                self.write_metrics(elapsed - ACHIEVED_HOLD_TIME, inside, avg_distance)
        else:
            self.achieved_since = None
        self.update_gui(inside, avg_distance, elapsed)

    def write_metrics(self, aggregation_time, inside, avg_distance):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.metrics_dir / f"{self.scenario}_metrics_{stamp}.csv"
        summary_path = self.metrics_dir / f"{self.scenario}_summary_{stamp}.txt"
        fieldnames = []
        for row in self.samples:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
        summary = [
            f"Scenario: {self.scenario}",
            f"Aggregation time: {aggregation_time:.3f} seconds",
            f"Robots in target region: {inside}/{ROBOT_COUNT}",
            f"Percentage in target region: {inside * 100.0 / ROBOT_COUNT:.2f}%",
            f"Average distance to target: {avg_distance:.4f} m",
            f"Collision count: {self.collision_count}",
            f"Total path length: {sum(self.path_lengths):.4f} m",
            f"CSV file: {csv_path.name}"
        ]
        summary_path.write_text("\n".join(summary) + "\n")
        self.metrics_written = True
        print("Metrics generated:", csv_path)
        print("Summary generated:", summary_path)

    def handle_keyboard(self):
        key = self.keyboard.getKey()
        while key != -1:
            if key == ord("1"):
                self.pending_scenario = "easy"
            elif key == ord("2"):
                self.pending_scenario = "medium"
            elif key == ord("3"):
                self.pending_scenario = "hard"
            key = self.keyboard.getKey()

    def run(self):
        while self.sup.step(self.time_step) != -1:
            self.update_gui()
            self.handle_keyboard()
            if self.pending_scenario:
                target = self.pending_scenario
                self.pending_scenario = None
                self.configure(target)
            self.collect_metrics()

if __name__ == "__main__":
    ScenarioSupervisor().run()