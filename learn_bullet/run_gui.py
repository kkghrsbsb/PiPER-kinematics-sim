import os
import time
import pybullet as p
import pybullet_data
from core.path import PIPER_DESCRIPTION_DIR

p.connect(p.GUI, options="--width=1280 --height=720")
p.setGravity(0, 0, -9.8)

# 从 pybullet_data 获取环境
plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
planeId = p.loadURDF(plane_path)

# 
robot_path = os.path.join(PIPER_DESCRIPTION_DIR, "urdf", "piper_description_v100_camera.urdf")
robotId = p.loadURDF(robot_path, [0, 0, 0], useFixedBase=True, globalScaling=5)

def blsleep(timer: float):
   delay_mark = time.time()
   while True:
       offset = time.time() - delay_mark
       if offset > timer:
           break
for i in range(10000):
   p.stepSimulation()
   blsleep(0.005)

p.disconnect()
