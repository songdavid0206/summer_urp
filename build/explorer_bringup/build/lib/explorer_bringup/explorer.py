import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_data = None
        self.goal_active = False
        self.timer = self.create_timer(3.0, self.explore)

    def map_callback(self, msg):
        self.map_data = msg

    def explore(self):
        if self.map_data is None or self.goal_active:
            return
        frontiers = self.detect_frontiers(self.map_data)
        if not frontiers:
            self.get_logger().info('No more frontiers!')
            return
        target = self.select_closest_frontier(frontiers)
        self.send_goal(target)

    def send_goal(self, target):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.get_logger().info(f'Sending goal to: {target}')
        self.goal_active = True
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            self.goal_active = False
            return
        self.get_logger().info('Goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Goal reached!')
        self.goal_active = False

    def detect_frontiers(self, map_msg):
        grid = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        frontiers = []
        for y in range(1, map_msg.info.height-1):
            for x in range(1, map_msg.info.width-1):
                if grid[y][x] == 0:
                    neighbors = grid[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        wx = x * map_msg.info.resolution + map_msg.info.origin.position.x
                        wy = y * map_msg.info.resolution + map_msg.info.origin.position.y
                        frontiers.append((wx, wy))
        return frontiers

    def select_closest_frontier(self, frontiers):
        # 실제로는 tf2로 로봇 위치를 받아와야 정확하지만, 예시로 (0,0) 기준
        robot_x, robot_y = 0.0, 0.0
        dists = [np.hypot(fx-robot_x, fy-robot_y) for fx, fy in frontiers]
        return frontiers[np.argmin(dists)]

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

