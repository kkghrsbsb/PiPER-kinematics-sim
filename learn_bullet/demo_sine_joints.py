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
        p.JOINT_REVOLUTE: "REVOLUTE",  # 转动关节
        p.JOINT_PRISMATIC: "PRISMATIC",  # 移动关节
        p.JOINT_SPHERICAL: "SPHERICAL",  # 球关节
        p.JOINT_PLANAR: "PLANAR",  # 平面关节
        p.JOINT_FIXED: "FIXED",  # 固定关节
        p.JOINT_POINT2POINT: "POINT2POINT",  # 点对点关节（球链约束）
        p.JOINT_GEAR: "GEAR",  # 齿轮/传动约束
    }
    return m.get(joint_type, f"UNKNOWN({joint_type})")


# 打印机器人所有关节的信息, 包括索引、名称、类型、限制和初始参数
def print_all_joints(robot_id: int) -> None:
    n = p.getNumJoints(robot_id)
    print(f"\n[Joint Summary] numJoints={n}\n")
    print(f"{'idx'}  {'j_name':<24s}  {'j_type':<10s}  {'limit':<22s}  {'q':<3s}  {'qd':<3s}")
    for j in range(n):
        info = p.getJointInfo(robot_id, j)
        # getJointInfo returns a tuple; key fields:
        # 0 jointIndex, 1 jointName, 2 jointType, 8 lowerLimit, 9 upperLimit
        idx = info[0]
        j_name = info[1].decode("utf-8")
        j_type = info[2]
        lower, upper = info[8], info[9]
        limit_str = f"[{lower:.6f}, {upper:.6f}]"

        # q: origin position(位置/角度)  qd: origin velocity(速度)
        q, qd = p.getJointState(robot_id, j)[:2]

        print(f"{idx:<3d}  {j_name:<24s}  {joint_type_name(j_type):<10s}  {limit_str:<22s}  {q}  {qd}")
    print("")


# 若使用 piper_description.urdf
#
# [Joint Summary] numJoints=9
#
# idx  j_name                    j_type      limit                   q    qd
# 0    joint1                    REVOLUTE    [-2.618000, 2.168000]   0.0  0.0
# 1    joint2                    REVOLUTE    [0.000000, 3.140000]    0.0  0.0
# 2    joint3                    REVOLUTE    [-2.967000, 0.000000]   0.0  0.0
# 3    joint4                    REVOLUTE    [-1.745000, 1.745000]   0.0  0.0
# 4    joint5                    REVOLUTE    [-1.220000, 1.220000]   0.0  0.0
# 5    joint6                    REVOLUTE    [-2.094400, 2.094400]   0.0  0.0
# 6    joint6_to_gripper_base    FIXED       [0.000000, -1.000000]   0.0  0.0
# 7    joint7                    PRISMATIC   [0.000000, 0.175000]    0.0  0.0
# 8    joint8                    PRISMATIC   [-0.175000, 0.000000]   0.0  0.0
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
    # 如果只有一个转动关节，后续对 j0/j1 做的操作都落在其同一个关节上
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
    # [Demo] Driving joints: 0(joint1), 1(joint2)

    # 电机控制参数
    dt = 1.0 / 240.0
    freq_hz = 0.25
    w = 2.0 * math.pi * freq_hz  # 角频率
    amp = 0.8  # 振幅 (按关节限制调整)
    phase = math.pi / 2.0  # 初相位
    max_force = 80  # N, 最大驱动力
    max_vel = math.pi  # rad/s, 最大关节速度

    simulation_time = 20.0

    for i in range(int(simulation_time * 240)):
        t = i * dt

        # 错限位, 运动更自然

        # joint1 limit=[-2.618000, 2.168000]
        target0 = amp * math.sin(w * t)

        # joint2 limit=[0.000000, 3.140000]
        # FIXME: 目标是正弦, 但关节有下限 0
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
