import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os
from datetime import datetime

class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller_node')
        
        # --- 1. SENSÖR VE VERİ DİNLEME ---
        # DÜZELTME: Artık 'robot/location' kanalını String (JSON) olarak dinliyoruz
        self.loc_sub = self.create_subscription(String, 'robot/location', self.location_callback, 10)
        
        # Yapay Zeka verisini dinleme
        self.ai_sub = self.create_subscription(String, 'ai/detections', self.ai_callback, 10)
        
        # --- DURUM DEĞİŞKENLERİ ---
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.location_ready = False
        
        # --- VERİ KAYIT (LOGLAMA) ALTYAPISI ---
        self.csv_file_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'robotanik_analiz_raporu.csv')
        self.init_csv_file()
        
        self.get_logger().info('Main Controller (Sistem Beyni) başlatıldı. Konum verisi bekleniyor...')

    def init_csv_file(self):
        file_exists = os.path.isfile(self.csv_file_path)
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(['Zaman', 'Hastalik_Turu', 'Risk_Skoru(%)', 'Yayilma_Orani(%)', 'Konum_X', 'Konum_Y'])

    def location_callback(self, msg):
        try:
            # Gelen String verisini JSON olarak çöz ve X, Y'yi kaydet
            loc_data = json.loads(msg.data)
            self.latest_x = loc_data.get('x', 0.0)
            self.latest_y = loc_data.get('y', 0.0)
            
            # İlk veri geldiğinde güvenlik duvarını kaldır
            if not self.location_ready:
                self.location_ready = True
                self.get_logger().info('✅ Konum verisi kilitlendi! Sistem tam otonomi/loglama için hazır.')
        except json.JSONDecodeError:
            pass

    def ai_callback(self, msg):
        # GÜVENLİK DUVARI: Konum yoksa loglama yapma
        if not self.location_ready:
            return

        try:
            data = json.loads(msg.data)
            label = data.get('label', 'Bilinmiyor')
            risk = data.get('risk_score', 0.0)
            spread = data.get('spread_ratio', 0.0)
            
            # Gerçek zamanı al
            current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # CSV'ye Yaz
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([current_time_str, label, f"{risk:.1f}", f"{spread:.1f}", f"{self.latest_x:.3f}", f"{self.latest_y:.3f}"])
                
            self.get_logger().info(f"[LOGLANDI] Hedef: {label} | Risk: %{risk:.1f} | Konum: (X:{self.latest_x:.2f}, Y:{self.latest_y:.2f})")

        except json.JSONDecodeError:
            self.get_logger().error('AI verisi JSON formatında çözülemedi!')

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
