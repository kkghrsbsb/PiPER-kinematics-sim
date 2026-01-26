# 对两个转动关节做正弦小幅运动(根据真实时间)
# setRealTimeSimulation(1) 模式
# 根据物理服务器的实时时钟自动运行正向动力学仿真

import os
import time
import math

import pybullet as p
import pybullet_data

from core.path import PIPER_DESCRIPTION_DIR
from learn_bullet.demo_sine_joints import print_all_joints, pick_two_revolute_joints


def main():
    p.connect(p.GUI, options="--width=1920 --height=1080")

    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)  # 先关，加载完再开，避免加载时抖动

    # Optional: nicer camera
    p.resetDebugVisualizerCamera(
        cameraDistance=1.8,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.3],
    )

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
    freq_hz = 0.25
    w = 2.0 * math.pi * freq_hz
    amp = 0.8
    phase = math.pi / 2.0
    max_force = 80
    max_vel = math.pi

    # 开启实时仿真: 之后不要再 stepSimulation()
    p.setRealTimeSimulation(1)

    # 用真实时间驱动正弦（用 perf_counter 更稳）
    t0 = time.perf_counter()

    simulation_time = 20.0

    # 需要给控制指令的频率（不是物理步进频率）
    period = 1.0 / 240.0
    next_tick = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            t = now - t0
            if t >= simulation_time:
                break

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

            # 控制循环节拍（不调用 stepSimulation）
            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 如果机器很卡，追不上节拍就直接跳过 sleep
                next_tick = time.perf_counter()
    finally:
        p.setRealTimeSimulation(0)
        p.disconnect()


if __name__ == "__main__":
    main()
