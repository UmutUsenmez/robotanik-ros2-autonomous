import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        
        self.location_buffer = [] 

        # --- YENİ: CSV LOGLAMA AYARLARI ---
        # Verilerin kaydedileceği dosyanın yolu
        self.csv_output_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_control/data/hastalik_haritasi_verileri.csv'
        if not os.path.exists(self.csv_output_path):
            with open(self.csv_output_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Zaman', 'X_Koordinati', 'Y_Koordinati', 'Hastalik_Adi'])
        # -----------------------------------

        self.loc_sub = self.create_subscription(String, 'robot/location', self.location_callback, 10)
        self.ai_sub = self.create_subscription(String, 'ai/detections', self.ai_callback, 10)
            
        self.get_logger().info('🧠 Main Controller devrede. Otonom CSV loglama aktif...')

    def location_callback(self, msg):
        loc_data = json.loads(msg.data)
        self.location_buffer.append(loc_data)
        if len(self.location_buffer) > 50:
            self.location_buffer.pop(0)

    def ai_callback(self, msg):
        ai_data = json.loads(msg.data)
        hastalik = ai_data["label"]
        ai_time = ai_data["time"]
        
        if hastalik != "Healthy" and len(self.location_buffer) > 0:
            
            closest_loc = min(self.location_buffer, key=lambda loc: abs(loc["time"] - ai_time))
            time_diff = abs(closest_loc["time"] - ai_time)
            
            self.get_logger().warning(
                f"🚨 {hastalik} TESPİT EDİLDİ! \n"
                f"   📍 Konum -> X: {closest_loc['x']:.3f}, Y: {closest_loc['y']:.3f}"
            )

            # --- YENİ: HASTALIĞI ANINDA CSV DOSYASINA YAZMA ---
            try:
                # Dosyayı 'a' (append/ekleme) modunda açıyoruz ki eski verileri silmesin, alt alta eklesin
                with open(self.csv_output_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    # Sütun sırasına göre veriyi yaz: Zaman, X, Y, Hastalık
                    writer.writerow([f"{ai_time:.2f}", closest_loc['x'], closest_loc['y'], hastalik])
                
                self.get_logger().info(f"💾 Veri başarıyla haritaya işlendi: {self.csv_output_path}")
            except Exception as e:
                self.get_logger().error(f"CSV Kayıt Hatası: {e}")
            # -------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
