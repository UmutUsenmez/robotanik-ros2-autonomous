#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import math
import json
from rclpy.parameter import Parameter  # <-- YENİ EKLENDİ

class RealLocationPublisher(Node):
    def __init__(self):
        # --- ŞEFİN SİMÜLASYON SENKRONİZASYON ZIRHI V2 (Humble Uyumlu) ---
        # Parametreyi sonradan ekleyip çakışma yaratmak yerine, 
        # Node doğarken genetiğine doğrudan enjekte ediyoruz!
        super().__init__(
            'real_location_publisher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        # ---------------------------------------------------------------
        
        self.publisher_ = self.create_publisher(String, 'robot/location', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🌍 Simülasyon Konum Yayıncısı Başladı! (Sim Time ile Senkronize)")

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            current_ros_time = self.get_clock().now().nanoseconds / 1e9

            loc_data = {
                "time": round(current_ros_time, 3),
                "x": round(x, 3),
                "y": round(y, 3),
                "theta": round(theta, 3)
            }

            msg = String()
            msg.data = json.dumps(loc_data)
            self.publisher_.publish(msg)

        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RealLocationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
