import rclpy
import sys
import threading
import time
import numpy as np

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.noise_odometry import NoisyOdometryNode
from hal_interfaces.general.laser import LaserNode

freq = 90.0

def __auto_spin() -> None:
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        time.sleep(1/freq)


if not rclpy.ok():
    rclpy.init(args=sys.argv)

### HAL INIT ###
motor_node = MotorsNode("/turtlebot3/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/turtlebot3/odom")
noisy_odometry_node = NoisyOdometryNode("/turtlebot3/odom")
laser_node = LaserNode("/turtlebot3/laser/scan")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node)
executor.add_node(laser_node)

executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


### GETTERS ### 

# Pose
def getPose3d():
    try:
        return odometry_node.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")  

# Pose
def getOdom():
    try:
        return noisy_odometry_node.getPose3d()
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")  

def getOdom2():
    real_pose = getPose3d()
    noisy_pose = getOdom()
    if real_pose.x - noisy_pose.x >= 0:
        noisy_pose.x = noisy_pose.x - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.x = noisy_pose.x + abs(np.random.normal(0.0, 0.1) * 0.01)
    if real_pose.y - noisy_pose.y >= 0:
        noisy_pose.y = noisy_pose.y - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.y = noisy_pose.y + abs(np.random.normal(0.0, 0.1) * 0.01)
    if real_pose.yaw - noisy_pose.yaw >= 0:
        noisy_pose.yaw = noisy_pose.yaw - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.yaw = noisy_pose.yaw + abs(np.random.normal(0.0, 0.1) * 0.01)
    return noisy_pose

def getOdom3():
    real_pose = getPose3d()
    noisy_pose = getOdom2()
    if real_pose.x - noisy_pose.x >= 0:
        noisy_pose.x = noisy_pose.x - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.x = noisy_pose.x + abs(np.random.normal(0.0, 0.1) * 0.01)
    if real_pose.y - noisy_pose.y >= 0:
        noisy_pose.y = noisy_pose.y - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.y = noisy_pose.y + abs(np.random.normal(0.0, 0.1) * 0.01)
    if real_pose.yaw - noisy_pose.yaw >= 0:
        noisy_pose.yaw = noisy_pose.yaw - abs(np.random.normal(0.0, 0.1) * 0.01)
    else:
        noisy_pose.yaw = noisy_pose.yaw + abs(np.random.normal(0.0, 0.1) * 0.01)
    return noisy_pose

def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data


### SETTERS ###

# Linear speed
def setV(v):
    motor_node.sendV(float(v))

# Angular speed
def setW(w):
    motor_node.sendW(float(w))