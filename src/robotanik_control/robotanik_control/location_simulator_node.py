import rclpy
from rclpy.node import Node
from std_msgs.msg import String # YENİ: JSON yayınlayacağımız için String yaptık
import csv
import json 

class LocationSimulatorNode(Node):
    def __init__(self):
        super().__init__('location_simulator_node')
        self.publisher_ = self.create_publisher(String, 'robot/location', 10)

        self.csv_file_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_control/data/sim_data.csv'
        self.data = []
        self.current_index = 0

        self.load_csv()

        # Excel'indeki time_s farkı 0.5 saniye olduğu için timer'ı da 0.5 yapıyoruz
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_csv(self):
        try:
            with open(self.csv_file_path, mode='r') as file:
                csv_reader = csv.reader(file) 
                next(csv_reader) 
                for row in csv_reader:
                    if len(row) == 1: 
                        row = row[0].split()
                    
                    if len(row) >= 4:
                        self.data.append({
                            'csv_time': float(row[0]),
                            'x': float(row[1]),
                            'y': float(row[2]),
                            'theta': float(row[3])
                        })
            self.get_logger().info(f'✅ Simülasyon verisi yüklendi! ({len(self.data)} konum)')
        except Exception as e:
            self.get_logger().error(f'🚨 CSV okuma hatası: {e}')

    def timer_callback(self):
        if self.current_index < len(self.data):
            current_data = self.data[self.current_index]
            current_ros_time = self.get_clock().now().nanoseconds / 1e9
            loc_data = {
                "time": current_ros_time, 
                "x": current_data['x'],
                "y": current_data['y'],
                "theta": current_data['theta']
            }

            msg = String()
            msg.data = json.dumps(loc_data)
            self.publisher_.publish(msg)
            
            self.current_index += 1
        else:
            self.current_index = 0 

def main(args=None):
    rclpy.init(args=args)
    node = LocationSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
