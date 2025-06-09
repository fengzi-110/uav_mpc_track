#!/usr/bin/env python

import rospy
import numpy as np
import osqp
from scipy import sparse
from scipy.interpolate import interp1d
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *

class CubeTracker:
    def __init__(self, model_name="target_cube", min_dist=0.02):
        self.model_name = model_name
        self.pose_history = []
        self.min_dist = min_dist
        self.last_pose = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def callback(self, msg):
        if self.model_name not in msg.name:
            return
        idx = msg.name.index(self.model_name)
        pos = msg.pose[idx].position
        p = self.gazebo_to_enu(np.array([pos.x, pos.y, pos.z]))
        if self.last_pose is None or np.linalg.norm(p - self.last_pose) > self.min_dist:
            self.pose_history.append(p)
            self.last_pose = p

    def get_interpolated_traj(self, step_dist=0.3):
        if len(self.pose_history) < 2:
            return []
        traj = np.array(self.pose_history)
        dists = np.sqrt(np.sum(np.diff(traj, axis=0)**2, axis=1))
        cumdist = np.hstack([[0], np.cumsum(dists)])
        uniform_dist = np.arange(0, cumdist[-1], step_dist)

        interp_x = interp1d(cumdist, traj[:, 0], kind='linear')
        interp_y = interp1d(cumdist, traj[:, 1], kind='linear')
        interp_z = interp1d(cumdist, traj[:, 2], kind='linear')
        resampled = np.vstack([interp_x(uniform_dist),
                               interp_y(uniform_dist),
                               interp_z(uniform_dist)]).T
        return resampled.tolist()

    def gazebo_to_enu(self, p):
        return np.array([-p[1], p[0], p[2]])


class MPCController:
    def __init__(self):
        self.dt = 0.1
        self.N = 10
        self.nx = 6
        self.nu = 3

        self.Q = sparse.diags([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])    
        self.R = 0.1 * sparse.eye(self.nu)

        I3 = np.eye(3)
        self.A = np.block([[I3, self.dt * I3], [np.zeros((3, 3)), I3]])
        self.B = np.block([[0.5 * self.dt ** 2 * I3], [self.dt * I3]])

        self.v_max = 2.0
        self.a_max = 2.0
        self.xmin = np.array([-np.inf]*3 + [-self.v_max]*3)
        self.xmax = np.array([ np.inf]*3 + [ self.v_max]*3)
        self.umin = -self.a_max * np.ones(3)
        self.umax =  self.a_max * np.ones(3)

        self.build_osqp_problem()

        self.current_state = np.zeros(6)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

        self.marker_pub = rospy.Publisher("/mpc_tracking_markers", Marker, queue_size=1)
        self.traj_line_pub = rospy.Publisher("/mpc_tracking_line", Marker, queue_size=1, latch=True)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.current_state = np.array([pos.x, pos.y, pos.z, vel.x, vel.y, vel.z])

    def build_osqp_problem(self):
        """
        state variable: x, v
        control variable: a
        
        min Sigma_{k=0}^{N-1} (x_k - x_k^ref)^T Q (x_k - x_k^ref) + u_k^T R u_k + (x_N - x_N^ref)^T Q (x_N - x_N^ref)

        """
        Q_bar = sparse.kron(sparse.eye(self.N), self.Q) 
        QN = self.Q
        R_bar = sparse.kron(sparse.eye(self.N), self.R)
        self.P = sparse.block_diag([Q_bar, QN, R_bar], format='csc')

        Ax = sparse.kron(sparse.eye(self.N+1), -sparse.eye(self.nx)) + \
             sparse.kron(sparse.eye(self.N+1, k=-1), self.A)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.B)
        self.Aeq = sparse.hstack([Ax, Bu])
        self.leq = np.zeros((self.N+1)*self.nx)
        self.ueq = np.zeros((self.N+1)*self.nx)

        self.Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        self.lineq = np.concatenate([
            np.tile(self.xmin, self.N+1),
            np.tile(self.umin, self.N)
        ])
        self.uineq = np.concatenate([
            np.tile(self.xmax, self.N+1),
            np.tile(self.umax, self.N)
        ])
        self.A_total = sparse.vstack([self.Aeq, self.Aineq], format='csc')
        self.prob = osqp.OSQP()

    def compute_reference(self, ref_traj):
        cur_pos = self.current_state[:3]
        traj_np = np.array(ref_traj)
        dists = np.linalg.norm(traj_np - cur_pos, axis=1)
        idx_start = np.argmin(dists)

        ref = []
        for i in range(self.N):
            idx = min(idx_start + i, len(ref_traj)-1)
            ref.append(np.hstack((ref_traj[idx], [0, 0, 0])))
        ref.append(ref[-1])
        return np.array(ref)

    def solve_mpc(self, ref_traj):
        Xr = self.compute_reference(ref_traj)
        q_parts = [-self.Q.dot(Xr[i]) for i in range(self.N)]
        q_vector = np.hstack(q_parts)
        q = np.hstack([q_vector, -self.Q.dot(Xr[-1]), np.zeros(self.N * self.nu)])

        leq = self.leq.copy()
        ueq = self.ueq.copy()
        leq[:self.nx] = -self.current_state
        ueq[:self.nx] = -self.current_state

        self.prob.setup(P=self.P, q=q, A=self.A_total,
                        l=np.hstack([leq, self.lineq]),
                        u=np.hstack([ueq, self.uineq]),
                        warm_start=True, verbose=False)

        res = self.prob.solve()
        if res.info.status != 'solved':
            raise ValueError("OSQP failed")
        u_opt = res.x[-self.N*self.nu:-(self.N-1)*self.nu]

        return u_opt, Xr    

    def publish_marker(self, ref_points):

        for i, p in enumerate(ref_points[:self.N]):  # red, reference points 
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "ref_points"
            marker.id = 100 + i  
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = p[0]
            marker.pose.position.y = p[1]
            marker.pose.position.z = p[2]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0    
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            self.marker_pub.publish(marker)

        pos_marker = Marker()
        pos_marker.header.frame_id = "map"
        pos_marker.header.stamp = rospy.Time.now()
        pos_marker.ns = "uav_tracking"
        pos_marker.id = 3
        pos_marker.type = Marker.SPHERE
        pos_marker.action = Marker.ADD
        pos_marker.pose.position.x = self.current_state[0]
        pos_marker.pose.position.y = self.current_state[1]
        pos_marker.pose.position.z = self.current_state[2]
        pos_marker.scale.x = 0.15
        pos_marker.scale.y = 0.15
        pos_marker.scale.z = 0.15
        pos_marker.color.b = 1.0     # blue, uav current position
        pos_marker.color.a = 1.0
        pos_marker.pose.orientation.w = 1.0
        self.marker_pub.publish(pos_marker)

    def publish_full_traj(self, traj):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "uav_tracking"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in traj]
        self.traj_line_pub.publish(marker)

    def run(self, get_traj_func):
        rate = rospy.Rate(1 / self.dt)
        rospy.loginfo(CBLUE2 + "MPC controller started." + CEND)

        while not rospy.is_shutdown():
            traj = get_traj_func()
            if len(traj) < self.N + 1:
                rospy.logwarn_throttle(2.0, "Trajectory too short. Waiting...")
                rate.sleep()
                continue

            self.publish_full_traj(traj)
            try:
                # u, ref_point = self.solve_mpc(traj)
                u, ref_traj_points = self.solve_mpc(traj)

            except Exception as e:
                rospy.logerr("MPC solve failed: {}".format(e))
                continue

            v_next = self.current_state[3:6] + u * self.dt   # approximate acceleration by integration
            rospy.loginfo(CYELLOW2 + "Velocity command: {:.2f}, {:.2f}, {:.2f}".format(*v_next) + CEND)

            cmd = TwistStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.twist.linear.x = v_next[0]
            cmd.twist.linear.y = v_next[1]
            cmd.twist.linear.z = v_next[2]
            self.pub.publish(cmd)

            self.publish_marker(ref_traj_points[:self.N, :3])
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("mpc_velocity_control")

    drone = gnc_api()
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(3)

    rospy.loginfo(CBLUE2 + "Tracking cube trajectory LIVE..." + CEND)
    tracker = CubeTracker("target_cube")
    mpc = MPCController()
    mpc.run(tracker.get_interpolated_traj)
