import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv
import numpy as np

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.pub_cmd = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)

        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0
        self.odom_received = False

        # 估计位姿
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_yaw = 0.0

        # 噪声标准差
        self.sigma_v = 0.05
        self.sigma_w = 0.02

        # 记录数据
        self.data = []
        self.start_time = self.get_clock().now()
        self.finished = False
        self.save_done = False

        # 轨迹参数
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # 状态机：0 直行，1 暂停，2 转弯，3 暂停，4 直行，5 暂停并永久停止
        self.phase = 0
        self.phase_start_time = self.get_clock().now()

        self.duration_straight = 4.0
        self.duration_turn = math.pi / 2 / 0.5
        self.duration_pause = 1.0
        self.v_straight = 0.5
        self.w_turn = 0.5

        self.get_logger().info('Localization node started. Waiting for odometry...')

    def odom_callback(self, msg):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.gt_yaw = quaternion_to_yaw(q)
        self.odom_received = True

        if self.finished:
            return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        self.data.append([t, self.gt_x, self.gt_y, self.est_x, self.est_y])

    def control_loop(self):
        if not self.odom_received:
            return

        now = self.get_clock().now()
        elapsed = (now - self.phase_start_time).nanoseconds / 1e9
        cmd = Twist()

        if self.phase == 0:          # 直行
            cmd.linear.x = self.v_straight
            cmd.angular.z = 0.0
            if elapsed >= self.duration_straight:
                self.phase = 1
                self.phase_start_time = now
        elif self.phase == 1:        # 暂停
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if elapsed >= self.duration_pause:
                self.phase = 2
                self.phase_start_time = now
        elif self.phase == 2:        # 转弯
            cmd.linear.x = 0.0
            cmd.angular.z = self.w_turn
            if elapsed >= self.duration_turn:
                self.phase = 3
                self.phase_start_time = now
        elif self.phase == 3:        # 暂停
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if elapsed >= self.duration_pause:
                self.phase = 4
                self.phase_start_time = now
        elif self.phase == 4:        # 再直行
            cmd.linear.x = self.v_straight
            cmd.angular.z = 0.0
            if elapsed >= self.duration_straight:
                self.phase = 5
                self.phase_start_time = now
        else:                        # phase 5: 永久停止
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if not self.finished and elapsed >= self.duration_pause:
                self.finished = True
                self.get_logger().info('Trajectory complete. Saving data...')
                self.save_data()

        # 发布速度指令
        self.pub_cmd.publish(cmd)

        # 更新估计位姿（带噪声）
        if not self.finished:
            v_noisy = cmd.linear.x + np.random.normal(0, self.sigma_v)
            w_noisy = cmd.angular.z + np.random.normal(0, self.sigma_w)

            self.est_x += v_noisy * math.cos(self.est_yaw) * self.dt
            self.est_y += v_noisy * math.sin(self.est_yaw) * self.dt
            self.est_yaw += w_noisy * self.dt

    def save_data(self):
        filename = 'data/trajectory_data.csv'
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'gt_x', 'gt_y', 'est_x', 'est_y'])
            writer.writerows(self.data)
        self.get_logger().info(f'Data saved to {filename}')

        if len(self.data) > 0:
            last = self.data[-1]
            error = math.sqrt((last[1] - last[3])**2 + (last[2] - last[4])**2)
            self.get_logger().info(f'Final position error: {error:.3f} m')
            with open('results/final_error.txt', 'w') as f:
                f.write(f'Final position error: {error:.3f} m\n')

def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
