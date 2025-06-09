#!/usr/bin/env python

# Import ROS
import rospy
# Import the API
from iq_gnc.py_gnc_functions import *
# For colored logs (optional)
from iq_gnc.PrintColours import *

def main():
    # 初始化 ROS 节点
    rospy.init_node("velocity_test_node", anonymous=True)

    # 创建 API 对象
    drone = gnc_api()

    # 连接 MAVROS
    drone.wait4connect()
    drone.wait4start()

    # 初始化坐标系并起飞
    drone.initialize_local_frame()
    drone.takeoff(3)

    # 设定发布频率（10Hz）
    rate = rospy.Rate(10)

    rospy.loginfo(CBLUE2 + "Publishing velocity command..." + CEND)

    # 向前飞 5 秒（vx = 1.0）
    t_start = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t_start < 5.0 and not rospy.is_shutdown():
        drone.send_velocity(1.0, 0.0, 0.0)  # 向前
        rate.sleep()

    # 停止运动
    rospy.loginfo(CYELLOW2 + "Stopping velocity..." + CEND)
    for _ in range(10):
        drone.send_velocity(0.0, 0.0, 0.0)
        rate.sleep()

    # 降落
    drone.land()
    rospy.loginfo(CGREEN2 + "Landing complete." + CEND)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
