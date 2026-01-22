import os
import time
import pybullet as p
import pybullet_data

p.connect(p.GUI, options="--width=1280 --height=720")
p.setGravity(0, 0, -9.8)

plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
planeId = p.loadURDF(plane_path)

ROOT = os.path.dirname(os.path.abspath(__file__))
robot_path = os.path.join(ROOT, "piper_description", "urdf", "piper_description_v100_camera.urdf")
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
