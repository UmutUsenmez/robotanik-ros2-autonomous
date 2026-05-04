import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os

class MainControllerNode(Node):
    def __init__(self): # DÜZELTİLDİ: __init__
        super().__init__('main_controller') # DÜZELTİLDİ: __init__
        
        self.location_buffer = [] 
        
        # DÜZELTİLDİ: Kullanıcı adı 'aziz' ve proje klasörü 'ros2v2' olarak güncellendi
        self.csv_output_path = '/home/umut/robotanik_guncel_ws/src/robotanik_control/data/hastalik_haritasi_verileri.csv'
        
        # Klasör yoksa oluştur
        os.makedirs(os.path.dirname(self.csv_output_path), exist_ok=True)
        
        # CSV Başlıklarını oluştur
        if not os.path.exists(self.csv_output_path):
            with open(self.csv_output_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Zaman', 'X_Koordinati', 'Y_Koordinati', 'Hastalik_Adi', 'Yayilma_Orani_%', 'Risk_Skoru'])

        self.loc_sub = self.create_subscription(String, 'robot/location', self.location_callback, 10)
        self.ai_sub = self.create_subscription(String, 'ai/detections', self.ai_callback, 10)
            
        self.get_logger().info('🧠 Main Controller devrede. Otonom CSV loglama aktif...')

    def location_callback(self, msg):
        try:
            loc_data = json.loads(msg.data)
            self.location_buffer.append(loc_data)
            # Hafızayı şişirmemek için son 100 veriyi tut
            if len(self.location_buffer) > 100:
                self.location_buffer.pop(0)
        except Exception as e:
            self.get_logger().error(f"Lokasyon JSON hatası: {e}")

    def ai_callback(self, msg):
        try:
            ai_data = json.loads(msg.data)
            hastalik = ai_data.get("label", "Healthy")
            ai_time = ai_data.get("time", 0.0)
            
            risk_skoru = ai_data.get("risk_score", 0.0)
            yayilma_orani = ai_data.get("spread_ratio", 0.0)
            
            if hastalik != "Healthy" and len(self.location_buffer) > 0:
                # Zaman damgasına göre en yakın konumu bul
                closest_loc = min(self.location_buffer, key=lambda loc: abs(loc.get("time", 0) - ai_time))
                
                self.get_logger().warning(
                    f"🚨 {hastalik} TESPİT EDİLDİ! (Risk: %{risk_skoru:.1f})\n"
                    f"   📍 Konum -> X: {closest_loc.get('x', 0):.3f}, Y: {closest_loc.get('y', 0):.3f}"
                )

                # CSV Kaydı
                with open(self.csv_output_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([
                        f"{ai_time:.2f}", 
                        closest_loc.get('x', 0), 
                        closest_loc.get('y', 0), 
                        hastalik, 
                        f"{yayilma_orani:.2f}", 
                        f"{risk_skoru:.2f}"
                    ])
                
                self.get_logger().info('🚨 Veri CSV dosyasına işlendi.')
        except Exception as e:
            self.get_logger().error(f"AI İşleme Hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': # DÜZELTİLDİ: __name__ ve __main__
    main()
