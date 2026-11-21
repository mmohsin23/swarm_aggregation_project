from controller import Supervisor
import math

sup = Supervisor()
time_step = int(sup.getBasicTimeStep())
TARGET_CENTER = (0,0)
TARGET_RADIUS = 0.5
COLLISION_DIST = 0.15

# Find all e-puck robots (by DEF names)
robot_nodes = []
for i in range(1,6):
    node = sup.getFromDef(f"robot{i}")
    if node:
        robot_nodes.append(node)

collision_total = 0
start_time = sup.getTime()
aggregation_time = None

with open("metrics.csv", "w") as f:
    f.write("time,inside_count,avg_distance,collisions\n")

while sup.step(time_step) != -1:
    positions = []
    inside = 0
    for node in robot_nodes:
        pos = node.getPosition()
        x, z = pos[0], pos[2]
        positions.append((x,z))
        if math.hypot(x, z) <= TARGET_RADIUS:
            inside += 1
    
    # collision count increment (pairwise)
    for i in range(len(positions)):
        for j in range(i+1, len(positions)):
            d = math.hypot(positions[i][0]-positions[j][0],
                           positions[i][1]-positions[j][1])
            if d < COLLISION_DIST:
                collision_total += 1
    
    avg_dist = sum(math.hypot(p[0],p[1]) for p in positions)/len(positions)
    current_time = sup.getTime()
    
    # Log every 0.5 seconds
    if int(current_time*2) % 1 == 0:
        with open("metrics.csv", "a") as f:
            f.write(f"{current_time},{inside},{avg_dist},{collision_total}\n")
    
    # Detect aggregation (80% inside)
    if aggregation_time is None and inside >= 0.8*len(robot_nodes):
        aggregation_time = current_time
        print(f"Aggregation achieved at {aggregation_time:.2f} seconds")
    
    # Print to console
    if current_time % 2.0 < 0.1:
        print(f"t={current_time:.1f}: inside={inside}/{len(robot_nodes)} avg_dist={avg_dist:.2f} collisions={collision_total}")