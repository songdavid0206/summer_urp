# dwa_local_planner.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np, math

class DwaLocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')
        self.scan = None
        self.vel = (0.0, 0.0)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('safety_margin', 0.05)
        self.declare_parameter('max_v', 0.2)
        self.declare_parameter('max_w', math.pi / 4)
        self.declare_parameter('alpha', 0.1)
        self.declare_parameter('beta', 0.7)

    def scan_cb(self, msg):
        self.scan = np.array(msg.ranges)

    def odom_cb(self, msg):
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def compute_cmd(self):
        if self.scan is None:
            print("[DEBUG] No scan data available.")
            return None, None

        max_v = self.get_parameter('max_v').get_parameter_value().double_value
        max_w = self.get_parameter('max_w').get_parameter_value().double_value
        r = self.get_parameter('robot_radius').get_parameter_value().double_value
        margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        α = self.get_parameter('alpha').get_parameter_value().double_value
        β = self.get_parameter('beta').get_parameter_value().double_value

        Vs = np.linspace(0.0, max_v, 5)
        Ws = np.linspace(-max_w, max_w, 11)
        best = (0.0, 0.0)
        best_score = -1e9
        dt = 0.1
        steps = 10

        collision_count = 0
        valid_trajectories = 0

        for v in Vs:
            for w in Ws:
                traj = self.predict(v, w, dt, steps)
                if self._collision(traj, r, margin):
                    collision_count += 1
                    continue
                valid_trajectories += 1
                score = self._eval(v, w, traj, α, β)
                print(f"[DEBUG] v={v:.2f}, w={w:.2f}, score={score:.3f}")
                if score > best_score:
                    best_score = score
                    best = (v, w)

        print(f"[DEBUG] Number of collisions skipped: {collision_count}")
        print(f"[DEBUG] Number of valid trajectories evaluated: {valid_trajectories}")
        print(f"[DEBUG] Best velocity selected: v={best[0]:.2f}, w={best[1]:.2f} with score={best_score:.3f}")

        return best
    
    def predict(self, v, w, dt, steps):
        pts = []
        x = y = θ = 0.0
        for _ in range(steps):
            x += v * math.cos(θ) * dt
            y += v * math.sin(θ) * dt
            θ += w * dt
            pts.append((x, y))
        return pts

    def _collision(self, traj, r, margin):
        for x, y in traj:
            θ = math.atan2(y, x)
            idx = int((θ + math.pi) / (2 * math.pi) * len(self.scan))
            if 0 <= idx < len(self.scan) and self.scan[idx] < r + margin:
                return True
        return False

    def _eval(self, v, w, traj, α, β):
        return α * (-abs(w)) + β * traj[-1][0]

