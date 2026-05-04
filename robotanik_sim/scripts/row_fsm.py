#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range as RangeMsg
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


class MissionState(Enum):
    STARTUP_WAIT       = 0
    SEND_GOAL          = 1
    WAIT_GOAL_RESPONSE = 2
    WAIT_RESULT        = 3
    WAIT_CANCEL        = 4
    BACKUP             = 5
    NEXT_WAYPOINT      = 6
    DONE               = 7
    EVACUATE           = 8
    ROW_FOLLOWING      = 9


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class RobotanikRowFSM(Node):
    def __init__(self):
        super().__init__("robotanik_row_fsm")
        self.get_logger().info("### ROBOTANIK FSM V24.0 (FİZİKSEL MESAFE ZIRHI EKLENDİ) BAŞLADI ###")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub    = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_x   = 0.0
        self.current_y   = 0.0

        self.front_center = None
        self.front_left   = None
        self.front_right  = None
        self.back         = None

        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(RangeMsg, "/sonar/front_center", self.front_center_cb, sonar_qos)
        self.create_subscription(RangeMsg, "/sonar/front_left",   self.front_left_cb,   sonar_qos)
        self.create_subscription(RangeMsg, "/sonar/front_right",  self.front_right_cb,  sonar_qos)
        self.create_subscription(RangeMsg, "/sonar/back",         self.back_cb,         sonar_qos)

        self.corridor_centers  = [10.45, 8.50, 6.50, 4.50, 2.50, 0.55]
        
        self.y_row_entry_bottom = 2.5   
        self.y_row_exit_bottom  = 1.5   
        self.y_row_entry_top    = 47.5  
        self.y_row_exit_top     = 48.5  

        self.waypoints  = self.generate_smart_waypoints()
        
        self.current_wp = 0
        self.state      = MissionState.STARTUP_WAIT

        self.goal_handle         = None
        self.send_goal_future    = None
        self.result_future       = None
        self.cancel_future       = None
        self._goal_accepted_time = None

        self.startup_time  = time.time()
        self.startup_delay = 15.0

        self.obstacle_counter         = 0
        self.obstacle_confirm_count   = 3
        self.controller_settle_time   = 1.0

        self.anchor_wall   = None
        self.anchor_target = 0.35
        self.blind_threshold = 1.0

        self.scrape_threshold = 0.18   
        self.scrape_speed     = 0.05   
        self.scrape_angular   = 0.8    

        self.backup_start_time   = None
        self.backup_duration     = 2.5
        self.backup_speed        = -0.10
        self.evacuate_speed      = -0.20
        self.evacuate_start_time = 0.0

        self.skipped_corridors = []

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Nav2 Sunucusu bekleniyor...")
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 hala hazır değil! Bekleniyor...")
        self.get_logger().info("Nav2 BAĞLANDI!")

    def generate_smart_waypoints(self):
        waypoints = []
        for i, x in enumerate(self.corridor_centers):
            going_up = (i % 2 == 0)
            if going_up:
                if i == 0:
                    waypoints.append((x, self.y_row_entry_bottom, 1.5708, "NAV2"))
                
                waypoints.append((x, self.y_row_exit_top, 1.5708, "ROW_FOLLOW"))
                
                if i < len(self.corridor_centers) - 1:
                    nx = self.corridor_centers[i + 1]
                    waypoints.append((nx, self.y_row_entry_top, -1.5708, "NAV2"))
            else:
                waypoints.append((x, self.y_row_exit_bottom, -1.5708, "ROW_FOLLOW"))
                
                if i < len(self.corridor_centers) - 1:
                    nx = self.corridor_centers[i + 1]
                    waypoints.append((nx, self.y_row_entry_bottom, 1.5708, "NAV2"))
        return waypoints

    def front_center_cb(self, msg):
        try: self.front_center = float(msg.range)
        except: self.front_center = None

    def front_left_cb(self, msg):
        try: self.front_left = float(msg.range)
        except: self.front_left = None

    def front_right_cb(self, msg):
        try: self.front_right = float(msg.range)
        except: self.front_right = None

    def back_cb(self, msg):
        try: self.back = float(msg.range)
        except: self.back = None

    def _valid(self, v):
        return v is not None and not math.isnan(v) and not math.isinf(v)

    def front_obstacle_seen(self):
        if self.state == MissionState.ROW_FOLLOWING:
            if self._valid(self.front_center):
                return self.front_center < 0.40
            return False
        return False

    def back_is_safe(self):
        if not self._valid(self.back):
            return True
        return self.back > 0.30

    def controller_settled(self):
        if self._goal_accepted_time is None:
            return False
        return time.time() - self._goal_accepted_time > self.controller_settle_time

    def publish_stop(self):
        self.publish_cmd_vel(0.0, 0.0)

    def publish_cmd_vel(self, speed, angular=0.0):
        msg = Twist()
        try:
            s = float(speed)
            a = float(angular)
            if math.isnan(s): s = 0.0
            if math.isnan(a): a = 0.0
            msg.linear.x  = s
            msg.angular.z = a
            self.cmd_pub.publish(msg)
        except Exception:
            pass 

    def skip_to_next_corridor(self):
        current_x = self.waypoints[self.current_wp][0]
        if current_x not in self.skipped_corridors:
            self.skipped_corridors.append(current_x)
            self.get_logger().warn(f"KORİDOR {current_x:.2f} ENGELLİ → atlandı.")

        for i in range(self.current_wp + 1, len(self.waypoints)):
            nx, ny, nyaw, ntype = self.waypoints[i]
            if abs(nx - current_x) > 0.5 and ntype == "NAV2":
                self.current_wp = i
                self.state = MissionState.SEND_GOAL
                return
        self.state = MissionState.DONE

    def send_nav2_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.state = MissionState.WAIT_GOAL_RESPONSE

    def send_current_goal(self):
        if self.current_wp >= len(self.waypoints):
            self.state = MissionState.DONE
            return

        x, y, yaw, wp_type = self.waypoints[self.current_wp]

        if wp_type == "ROW_FOLLOW":
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.anchor_wall = None  
            self.state = MissionState.ROW_FOLLOWING
            self.get_logger().info(f"NAV2 UYKUDA — HİBRİT PİLOT DEVREDE. Hedef Y: {y}")
            return

        self.send_nav2_goal(x, y, yaw)

    def handle_goal_response(self):
        if self.send_goal_future.done():
            self.goal_handle = self.send_goal_future.result()
            if not self.goal_handle.accepted:
                self.state = MissionState.NEXT_WAYPOINT
                return
            self._goal_accepted_time = time.time()
            self.result_future = self.goal_handle.get_result_async()
            self.state = MissionState.WAIT_RESULT

    def loop(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
        except:
            pass

        if self.state == MissionState.STARTUP_WAIT:
            if time.time() - self.startup_time >= self.startup_delay:
                self.state = MissionState.SEND_GOAL

        elif self.state == MissionState.SEND_GOAL:
            self.send_current_goal()

        elif self.state == MissionState.WAIT_GOAL_RESPONSE:
            self.handle_goal_response()

        elif self.state == MissionState.WAIT_RESULT:
            if self.controller_settled():
                if self.front_obstacle_seen():
                    self.obstacle_counter += 1
                    if self.obstacle_counter >= self.obstacle_confirm_count:
                        self.publish_stop()
                        self.obstacle_counter = 0
                        if self.goal_handle:
                            self.cancel_future = self.goal_handle.cancel_goal_async()
                            self.goal_handle   = None
                            self.state = MissionState.WAIT_CANCEL
                        else:
                            self.backup_start_time = time.time()
                            self.state = MissionState.BACKUP
                else:
                    self.obstacle_counter = 0

            # --- ŞEFİN FİZİKSEL MESAFE ZIRHI (FAILSAFE) ---
            # Eğer ROS 2 mesajı yutar ve future'ı tamamlamazsa, robot hedefe 40cm yaklaştığında zorla geçiş yap!
            target_x, target_y, _, _ = self.waypoints[self.current_wp]
            dist = math.hypot(self.current_x - target_x, self.current_y - target_y)
            
            if dist < 0.40:
                self.get_logger().info("Hedefe fiziksel olarak yaklaşıldı -> Failsafe ile görev atlanıyor!")
                self.state = MissionState.NEXT_WAYPOINT
                return

            # Normal şartlarda çalışan action client dinleyicisi
            if self.result_future and self.result_future.done():
                self.state = MissionState.NEXT_WAYPOINT

        elif self.state == MissionState.ROW_FOLLOWING:
            
            l_raw = self.front_left  if self._valid(self.front_left)  else 1.5
            r_raw = self.front_right if self._valid(self.front_right) else 1.5
            c_raw = self.front_center if self._valid(self.front_center) else 1.5

            if c_raw < 0.40 and (l_raw < 0.5 or r_raw < 0.5):
                self.get_logger().warn(f"ROBOT YAN DÖNDÜ ({c_raw:.2f}) -> Durup yola hizalanıyor!")
                angular_z = 0.5 if l_raw > r_raw else -0.5
                self.publish_cmd_vel(0.0, angular_z)
                return

            if self.front_obstacle_seen():
                self.obstacle_counter += 1
                if self.obstacle_counter >= self.obstacle_confirm_count:
                    self.get_logger().error("KORİDOR TIKALI → TAHLİYE")
                    self.obstacle_counter = 0
                    self.publish_stop()
                    self.evacuate_start_time = time.time()
                    self.state = MissionState.EVACUATE
                    return
            else:
                self.obstacle_counter = 0

            _, target_y, yaw, _ = self.waypoints[self.current_wp]
            going_up = (yaw > 0)
            
            reached_exit = False
            if going_up and self.current_y >= target_y:
                reached_exit = True
            elif not going_up and self.current_y <= target_y:
                reached_exit = True

            if reached_exit:
                self.publish_stop()
                self.get_logger().info(f"KORİDOR BİTTİ (Y={self.current_y:.2f}) → NAV2 S-KAVİSİ İÇİN UYANDIRILIYOR")
                self.state = MissionState.NEXT_WAYPOINT
                return

            if r_raw < self.scrape_threshold:
                self.publish_cmd_vel(self.scrape_speed, self.scrape_angular)
                return

            if l_raw < self.scrape_threshold:
                self.publish_cmd_vel(self.scrape_speed, -self.scrape_angular)
                return

            Kp = 1.0
            l_blind = (l_raw > self.blind_threshold)
            r_blind = (r_raw > self.blind_threshold)

            if l_blind and r_blind:
                self.anchor_wall = None
                error = 0.0
            elif not l_blind and not r_blind:
                self.anchor_wall = None
                error = (l_raw - r_raw) / max(l_raw + r_raw, 0.1)
            elif l_blind:
                self.anchor_wall = 'right'
                error = self.anchor_target - r_raw
            else:
                self.anchor_wall = 'left'
                error = l_raw - self.anchor_target

            angular_z = max(-0.5, min(0.5, Kp * error))
            self.publish_cmd_vel(0.20, angular_z) 

        elif self.state == MissionState.WAIT_CANCEL:
            if self.cancel_future and self.cancel_future.done():
                self.backup_start_time = time.time()
                self.state = MissionState.BACKUP

        elif self.state == MissionState.BACKUP:
            if not self.back_is_safe():
                self.publish_stop()
                self.skip_to_next_corridor()
                return
            if time.time() - self.backup_start_time < self.backup_duration:
                self.publish_cmd_vel(self.backup_speed)
            else:
                self.publish_stop()
                self.skip_to_next_corridor()

        elif self.state == MissionState.EVACUATE:
            if not self.back_is_safe():
                if time.time() - self.evacuate_start_time > 10.0:
                    self.skip_to_next_corridor()
                    return
                self.publish_stop()
                return

            self.publish_cmd_vel(self.evacuate_speed)
            _, _, yaw, _ = self.waypoints[self.current_wp]
            if (yaw > 0  and self.current_y < self.y_approach_bottom + 0.5) or \
               (yaw < 0  and self.current_y > self.y_approach_top    - 0.5):
                self.publish_stop()
                self.skip_to_next_corridor()

        elif self.state == MissionState.NEXT_WAYPOINT:
            self.current_wp += 1
            self.state = MissionState.SEND_GOAL if self.current_wp < len(self.waypoints) else MissionState.DONE

        elif self.state == MissionState.DONE:
            self.publish_stop()


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
        
    node = RobotanikRowFSM()
    node.get_logger().info("ZIRHLI FSM DÖNGÜSÜ AKTİF.")
    
    while rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except RuntimeError as e:
            node.get_logger().warn(f"ROS 2 İletişim Hatası Yutuldu: {e}")
        except KeyboardInterrupt:
            break
            
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
