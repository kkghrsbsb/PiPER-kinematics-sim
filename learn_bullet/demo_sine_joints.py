# 对两个转动关节做正弦小幅运动
# stepSimulation() 单步正向运动学

import os
import time
import math

import pybullet as p
import pybullet_data

from core.path import PIPER_DESCRIPTION_DIR

# 将关节类型（整数）转换为可读的字符串名称
def joint_type_name(joint_type: int) -> str:
    m = {
        p.JOINT_REVOLUTE: "REVOLUTE",        # 转动关节
        p.JOINT_PRISMATIC: "PRISMATIC",      # 移动关节
        p.JOINT_SPHERICAL: "SPHERICAL",      # 球关节
        p.JOINT_PLANAR: "PLANAR",            # 平面关节
        p.JOINT_FIXED: "FIXED",              # 固定关节
        p.JOINT_POINT2POINT: "POINT2POINT",  # 点对点关节（球链约束）
        p.JOINT_GEAR: "GEAR",                # 齿轮/传动约束
    }
    return m.get(joint_type, f"UNKNOWN({joint_type})")


# 打印机器人所有关节的信息，包括索引、名称、类型和限制
def print_all_joints(robot_id: int) -> None:
    n = p.getNumJoints(robot_id)
    print(f"\n[Joint Summary] numJoints={n}\n")
    for j in range(n):
        info = p.getJointInfo(robot_id, j)
        # getJointInfo returns a tuple; key fields:
        #  0 jointIndex, 1 jointName, 2 jointType, 8 lowerLimit, 9 upperLimit
        idx = info[0]
        name = info[1].decode("utf-8")
        jtype = info[2]
        lower, upper = info[8], info[9]

        # For FIXED joints, limits are often (0, -1) or similar; keep printing anyway.
        print(
            f"idx={idx:2d}  name={name:<24s}  type={joint_type_name(jtype):<10s}  limit=[{lower:.6f}, {upper:.6f}]"
        )
    print("")

# 若使用 piper_description.urdf
#
# idx= 0  name=joint1                    type=REVOLUTE    limit=[-2.618000, 2.168000]
# idx= 1  name=joint2                    type=REVOLUTE    limit=[0.000000, 3.140000]
# idx= 2  name=joint3                    type=REVOLUTE    limit=[-2.967000, 0.000000]
# idx= 3  name=joint4                    type=REVOLUTE    limit=[-1.745000, 1.745000]
# idx= 4  name=joint5                    type=REVOLUTE    limit=[-1.220000, 1.220000]
# idx= 5  name=joint6                    type=REVOLUTE    limit=[-2.094400, 2.094400]
# idx= 6  name=joint6_to_gripper_base    type=FIXED       limit=[0.000000, -1.000000]
# idx= 7  name=joint7                    type=PRISMATIC   limit=[0.000000, 0.175000]
# idx= 8  name=joint8                    type=PRISMATIC   limit=[-0.175000, 0.000000]
#

# 选择两个转动关节
def pick_two_revolute_joints(robot_id: int):
    revolutes = []
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        if info[2] == p.JOINT_REVOLUTE:
            revolutes.append(j)
    if len(revolutes) == 0:
        raise RuntimeError("No REVOLUTE joints found.")
    j0 = revolutes[0]
    j1 = revolutes[1] if len(revolutes) > 1 else revolutes[0]
    return j0, j1


def main():
    p.connect(p.GUI, options="--width=1920 --height=1080")
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)

    # Optional: nicer camera
    p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, 0, 0.3])

    # Plane
    plane_path = os.path.join(pybullet_data.getDataPath(), "plane.urdf")
    p.loadURDF(plane_path)

    # Robot
    robot_path = os.path.join(PIPER_DESCRIPTION_DIR, "urdf", "piper_description.urdf")
    robot_id = p.loadURDF(robot_path, [0, 0, 0], useFixedBase=True, globalScaling=5)

    # 打印 joint 信息: index/name/type/limit
    print_all_joints(robot_id)

    # 选两个 revolute joint 做正弦驱动
    j0, j1 = pick_two_revolute_joints(robot_id)
    j0_name = p.getJointInfo(robot_id, j0)[1].decode("utf-8")
    j1_name = p.getJointInfo(robot_id, j1)[1].decode("utf-8")
    print(f"[Demo] Driving joints: {j0}({j0_name}), {j1}({j1_name})\n")

    # 电机控制参数
    dt = 1.0 / 240.0
    freq_hz = 0.25                 # sine frequency
    w = 2.0 * math.pi * freq_hz
    amp = 0.8                      # radians (adjust if your joint limits are tight)
    phase = math.pi / 2.0
    max_force = 80                 # increase if it barely moves
    max_vel = 2.0                  # rad/s, optional

    simulation_time = 20.0

    for i in range(int(simulation_time * 240)):
        t = i * dt

        target0 = amp * math.sin(w * t)
        target1 = 0.6 * amp * math.sin(w * t + phase)

        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target0,
            force=max_force,
            maxVelocity=max_vel,
        )
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target1,
            force=max_force,
            maxVelocity=max_vel,
        )

        p.stepSimulation()
        time.sleep(dt)

    p.disconnect()


if __name__ == "__main__":
    main()
