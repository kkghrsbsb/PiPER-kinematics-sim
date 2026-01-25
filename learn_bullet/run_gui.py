# 运行 PyBullet GUI 并加载 Piper 机器人模型
# stepSimulation() 单步正向运动学

import os
import time
import pybullet as p
import pybullet_data
from core.path import PIPER_DESCRIPTION_DIR

p.connect(p.GUI, options="--width=1920 --height=1080")
p.setGravity(0, 0, -9.8)

# Plane
plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
planeId = p.loadURDF(plane_path)

# Robot
robot_path = os.path.join(PIPER_DESCRIPTION_DIR, "urdf", "piper_description.urdf")
robotId = p.loadURDF(robot_path, [0, 0, 0], useFixedBase=True, globalScaling=5)

simulation_time = 100.0  # 设置仿真时间(秒)
for i in range(int(simulation_time * 240)):
   p.stepSimulation()
   time.sleep(1./240.)  # 240 Hz

p.disconnect()
