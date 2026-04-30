#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from nav2_msgs.action import NavigateToPose


class MissionState(Enum):
    STARTUP_WAIT = 0
    SEND_GOAL = 1
    WAIT_GOAL_RESPONSE = 2
    WAIT_RESULT = 3
    CANCEL_GOAL = 4
    WAIT_CANCEL = 5
    BACKUP = 6
    RETRY = 7
    NEXT_WAYPOINT = 8
    DONE = 9


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class RobotanikRowFSM(Node):
    def __init__(self):
        super().__init__("robotanik_row_fsm")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.front_center = None
        self.front_left = None
        self.front_right = None
        self.back = None

        self.create_subscription(Range, "/sonar/front_center", self.front_center_cb, 10)
        self.create_subscription(Range, "/sonar/front_left", self.front_left_cb, 10)
        self.create_subscription(Range, "/sonar/front_right", self.front_right_cb, 10)
        self.create_subscription(Range, "/sonar/back", self.back_cb, 10)

        # Sağ dış koridordan başlıyoruz. Launch spawn:
        # x=10.45, y=1.0, yaw=+1.5708 olmalı.
        self.corridor_centers = [10.45, 8.50, 6.50, 4.50, 2.50, 0.55]

        self.y_scan_min = 2.70
        self.y_scan_max = 47.30
        self.y_bottom_turn = 1.20
        self.y_top_turn = 48.80

        self.waypoints = self.generate_serpentine_waypoints()

        self.current_wp = 0
        self.state = MissionState.STARTUP_WAIT

        self.goal_handle = None
        self.send_goal_future = None
        self.result_future = None
        self.cancel_future = None

        self.startup_time = time.time()
        self.startup_delay = 6.0

        self.obstacle_counter = 0
        self.obstacle_confirm_count = 5

        self.front_obstacle_threshold = 0.45
        self.back_obstacle_threshold = 0.30

        self.retry_count = 0
        self.max_retry_per_waypoint = 1

        self.backup_start_time = None
        self.backup_duration = 2.0
        self.backup_speed = -0.08

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("Robotanik Row FSM başlatıldı.")
        self.get_logger().info("FSM çalışırken RViz'den manuel goal verme.")
        self.get_logger().info(f"Toplam waypoint sayısı: {len(self.waypoints)}")

        for i, wp in enumerate(self.waypoints):
            x, y, yaw = wp
            self.get_logger().info(
                f"WP {i + 1:02d}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )

        self.get_logger().info("Nav2 navigate_to_pose action server bekleniyor...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server bulundu. Başlangıç gecikmesi bekleniyor.")

    def generate_serpentine_waypoints(self):
        waypoints = []

        for i, x in enumerate(self.corridor_centers):
            going_up = (i % 2 == 0)

            if going_up:
                waypoints.append((x, self.y_bottom_turn, 1.5708))
                waypoints.append((x, self.y_scan_min, 1.5708))
                waypoints.append((x, self.y_scan_max, 1.5708))
                waypoints.append((x, self.y_top_turn, 0.0))

                if i < len(self.corridor_centers) - 1:
                    next_x = self.corridor_centers[i + 1]
                    waypoints.append((next_x, self.y_top_turn, 0.0))
                    waypoints.append((next_x, self.y_top_turn, -1.5708))
                    waypoints.append((next_x, self.y_scan_max, -1.5708))

            else:
                waypoints.append((x, self.y_top_turn, -1.5708))
                waypoints.append((x, self.y_scan_max, -1.5708))
                waypoints.append((x, self.y_scan_min, -1.5708))
                waypoints.append((x, self.y_bottom_turn, 0.0))

                if i < len(self.corridor_centers) - 1:
                    next_x = self.corridor_centers[i + 1]
                    waypoints.append((next_x, self.y_bottom_turn, 0.0))
                    waypoints.append((next_x, self.y_bottom_turn, 1.5708))
                    waypoints.append((next_x, self.y_scan_min, 1.5708))

        return waypoints

    def front_center_cb(self, msg: Range):
        self.front_center = msg.range

    def front_left_cb(self, msg: Range):
        self.front_left = msg.range

    def front_right_cb(self, msg: Range):
        self.front_right = msg.range

    def back_cb(self, msg: Range):
        self.back = msg.range

    def front_obstacle_seen(self) -> bool:
        values = []

        for value in [self.front_center, self.front_left, self.front_right]:
            if value is not None and not math.isnan(value):
                values.append(value)

        if not values:
            return False

        return min(values) < self.front_obstacle_threshold

    def back_is_safe(self) -> bool:
        if self.back is None or math.isnan(self.back):
            return True

        return self.back > self.back_obstacle_threshold

    def send_current_goal(self):
        if self.current_wp >= len(self.waypoints):
            self.state = MissionState.DONE
            return

        x, y, yaw = self.waypoints[self.current_wp]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Waypoint {self.current_wp + 1}/{len(self.waypoints)} gönderiliyor: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        self.goal_handle = None
        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.state = MissionState.WAIT_GOAL_RESPONSE

    def handle_goal_response(self):
        if self.send_goal_future is None or not self.send_goal_future.done():
            return

        try:
            self.goal_handle = self.send_goal_future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal gönderme hatası: {exc}")
            self.state = MissionState.RETRY
            return

        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal reddedildi. Sonraki waypoint'e geçiliyor.")
            self.state = MissionState.NEXT_WAYPOINT
            return

        self.get_logger().info("Goal kabul edildi.")
        self.result_future = self.goal_handle.get_result_async()
        self.state = MissionState.WAIT_RESULT

    def request_cancel_goal(self):
        if self.goal_handle is None:
            self.get_logger().warn("İptal edilecek aktif goal yok. Backup'a geçiliyor.")
            self.state = MissionState.BACKUP
            self.backup_start_time = time.time()
            return

        self.get_logger().warn("Mevcut Nav2 goal iptal ediliyor.")
        self.cancel_future = self.goal_handle.cancel_goal_async()
        self.state = MissionState.WAIT_CANCEL

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_backup(self):
        msg = Twist()
        msg.linear.x = self.backup_speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def loop(self):
        if self.state == MissionState.STARTUP_WAIT:
            if time.time() - self.startup_time >= self.startup_delay:
                self.get_logger().info("Başlangıç gecikmesi tamamlandı. İlk waypoint gönderiliyor.")
                self.state = MissionState.SEND_GOAL
            return

        if self.state == MissionState.SEND_GOAL:
            self.send_current_goal()
            return

        if self.state == MissionState.WAIT_GOAL_RESPONSE:
            self.handle_goal_response()
            return

        if self.state == MissionState.WAIT_RESULT:
            if self.front_obstacle_seen():
                self.obstacle_counter += 1
            else:
                self.obstacle_counter = 0

            if self.obstacle_counter >= self.obstacle_confirm_count:
                self.get_logger().warn("Ön engel doğrulandı. Goal iptal edilecek.")
                self.obstacle_counter = 0
                self.publish_stop()
                self.state = MissionState.CANCEL_GOAL
                return

            if self.result_future is not None and self.result_future.done():
                try:
                    result = self.result_future.result()
                    self.get_logger().info(f"Waypoint sonucu status={result.status}")
                except Exception as exc:
                    self.get_logger().warn(f"Result okunamadı: {exc}")

                self.retry_count = 0
                self.state = MissionState.NEXT_WAYPOINT
                return

            return

        if self.state == MissionState.CANCEL_GOAL:
            self.request_cancel_goal()
            return

        if self.state == MissionState.WAIT_CANCEL:
            if self.cancel_future is not None and self.cancel_future.done():
                self.get_logger().info("Goal iptal sonucu alındı. Backup başlayacak.")
                self.publish_stop()
                self.backup_start_time = time.time()
                self.state = MissionState.BACKUP
            return

        if self.state == MissionState.BACKUP:
            if not self.back_is_safe():
                self.get_logger().warn("Arka güvenli değil. Backup durduruldu.")
                self.publish_stop()
                self.state = MissionState.RETRY
                return

            elapsed = time.time() - self.backup_start_time

            if elapsed < self.backup_duration:
                self.publish_backup()
            else:
                self.publish_stop()
                self.state = MissionState.RETRY

            return

        if self.state == MissionState.RETRY:
            self.retry_count += 1

            if self.retry_count <= self.max_retry_per_waypoint:
                self.get_logger().info(
                    f"Aynı waypoint tekrar deneniyor. Retry {self.retry_count}/{self.max_retry_per_waypoint}"
                )
                self.state = MissionState.SEND_GOAL
            else:
                self.get_logger().warn("Waypoint engelli kabul edildi. Sonraki waypoint'e geçiliyor.")
                self.retry_count = 0
                self.state = MissionState.NEXT_WAYPOINT

            return

        if self.state == MissionState.NEXT_WAYPOINT:
            self.current_wp += 1

            if self.current_wp >= len(self.waypoints):
                self.state = MissionState.DONE
            else:
                self.state = MissionState.SEND_GOAL

            return

        if self.state == MissionState.DONE:
            self.publish_stop()
            self.get_logger().info("Görev tamamlandı.")
            self.state = MissionState.STARTUP_WAIT
            self.startup_delay = 999999.0
            self.startup_time = time.time()
            return


def main(args=None):
    rclpy.init(args=args)
    node = RobotanikRowFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()