from controller import Supervisor, Keyboard
import math

TIME_STEP = 64
TARGET_RADIUS = 0.8

# Positions for each scenario (easy, medium, hard)
SCENARIOS = {
    1: {  # Easy
        "robot1": (-1.2, 0.8),
        "robot2": (1.1, -0.9),
        "robot3": (1.3, 1.2),
        "robot4": (-1.0, -1.1),
        "robot5": (0.5, -1.3),
    },
    2: {  # Medium
        "robot1": (-2.0, 1.5),
        "robot2": (2.0, -1.8),
        "robot3": (1.8, 2.0),
        "robot4": (-1.8, -2.0),
        "robot5": (2.0, 1.5),
    },
    3: {  # Hard
        "robot1": (-2.8, 2.5),
        "robot2": (2.8, -2.5),
        "robot3": (2.5, 2.8),
        "robot4": (-2.5, -2.8),
        "robot5": (0.0, 3.0),
    }
}

class SupervisorController:
    def __init__(self):
        self.supervisor = Supervisor()
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(TIME_STEP)
        self.robot_nodes = {}
        for name in ["robot1","robot2","robot3","robot4","robot5"]:
            node = self.supervisor.getFromDef(name)
            if node:
                self.robot_nodes[name] = node
        self.current_scenario = 1
        self.load_scenario(1)
        self.aggregation_achieved = False
        self.log_file = "aggregation_metrics.csv"
        try:
            with open(self.log_file, 'x') as f:
                f.write("scenario,aggregation_time_seconds\n")
        except FileExistsError:
            pass
        self.print_timer = 0

    def load_scenario(self, scenario_num):
        positions = SCENARIOS.get(scenario_num)
        if not positions:
            return
        for name, (x, z) in positions.items():
            node = self.robot_nodes.get(name)
            if node:
                node.getField("translation").setSFVec3f([x, z, 0.1])
        print(f"Loaded scenario {scenario_num} (1=Easy,2=Medium,3=Hard)")

    def reset_simulation(self):
        self.supervisor.simulationResetPhysics()
        self.load_scenario(self.current_scenario)
        self.supervisor.simulationReset()
        self.aggregation_achieved = False

    def all_inside(self):
        for name, node in self.robot_nodes.items():
            pos = node.getPosition()
            if math.hypot(pos[0], pos[2]) > TARGET_RADIUS:
                return False
        return True

    def log_metrics(self):
        if self.aggregation_achieved:
            return
        if self.all_inside():
            self.aggregation_achieved = True
            t = self.supervisor.getTime()
            with open(self.log_file, "a") as f:
                f.write(f"{self.current_scenario},{t:.2f}\n")
            print(f"*** Scenario {self.current_scenario} aggregated at {t:.2f} sec ***")

    def run(self):
        while self.supervisor.step(TIME_STEP) != -1:
            key = self.keyboard.getKey()
            if key == ord('1') and self.current_scenario != 1:
                self.current_scenario = 1
                self.reset_simulation()
            elif key == ord('2') and self.current_scenario != 2:
                self.current_scenario = 2
                self.reset_simulation()
            elif key == ord('3') and self.current_scenario != 3:
                self.current_scenario = 3
                self.reset_simulation()
            t = self.supervisor.getTime()
            if t - self.print_timer > 30:
                self.print_timer = t
                print("Press 1 (Easy), 2 (Medium), or 3 (Hard) to change scenario.")
            self.log_metrics()

if __name__ == "__main__":
    sup = SupervisorController()
    sup.run()