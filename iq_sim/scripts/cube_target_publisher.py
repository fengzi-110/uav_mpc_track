#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class CubeTargetPublisher:
    def __init__(self):
        rospy.init_node('cube_target_publisher', anonymous=True)
        self.pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.rate = rospy.Rate(10)  # 10 Hz

    def callback(self, data):
        try:
            idx = data.name.index("target_cube")  
        except ValueError:
            rospy.logwarn("target_cube not found in Gazebo model_states.")
            return

        pose = data.pose[idx]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose = pose
        self.pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"Published target pose at x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}")



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CubeTargetPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass

