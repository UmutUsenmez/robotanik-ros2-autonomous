import rclpy
from rclpy.node import Node
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import warnings

class HeatmapNode(Node):
    def __init__(self):
        super().__init__('heatmap_node')
        
        # SADECE OKUNACAK CSV DOSYASI
        self.csv_file_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'robotanik_analiz_raporu.csv')
        
        # ÇIKTI DOSYALARI (Sadece Görsel ve PDF)
        self.output_png_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'robotanik_canli_harita.png')
        self.output_pdf_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'robotanik_canli_harita.pdf')
        
        # Canlı harita ayarları
        plt.ion()
        self.fig = plt.figure(figsize=(12, 8)) # Yazıların sığması için haritayı biraz büyüttük
        
        self.timer = self.create_timer(3.0, self.update_heatmap)
        self.get_logger().info('🗺️ Canlı Harita Başlatıldı! (Yeşil: Sağlıklı, Renk Skalası: Riskli, Çıktı: PDF+PNG)')

    def update_heatmap(self):
        if not os.path.isfile(self.csv_file_path):
            return
            
        try:
            df = pd.read_csv(self.csv_file_path)
            
            if len(df) == 0:
                return
                
            plt.clf() 
            warnings.filterwarnings("ignore", category=UserWarning)
            
            # Verileri "Sağlıklı" ve "Hastalıklı" olarak ikiye ayır
            df['Hastalik_Turu_Lower'] = df['Hastalik_Turu'].str.lower()
            df_healthy = df[df['Hastalik_Turu_Lower'] == 'healthy'].copy()
            df_disease = df[df['Hastalik_Turu_Lower'] != 'healthy'].copy()
            
            # DİNAMİK EKSEN HESAPLAMASI (Tüm noktaları kapsayacak şekilde)
            min_x = df['Konum_X'].min()
            max_x = df['Konum_X'].max()
            min_y = df['Konum_Y'].min()
            max_y = df['Konum_Y'].max()
            
            plt.xlim(min_x - 2.0, max_x + 2.0)
            plt.ylim(min_y - 2.0, max_y + 2.0)
            
            # 1. HASTALIKLI NOKTALAR VE ISITMA BULUTU
            if len(df_disease) > 0:
                # Kırmızı tehlike bulutu
                sns.kdeplot(
                    x=df_disease['Konum_X'], 
                    y=df_disease['Konum_Y'], 
                    cmap="Reds", 
                    fill=True, 
                    thresh=0.05, 
                    alpha=0.3
                )
                
                # Risk seviyesine (Risk_Skoru) göre renklendirilen hastalık noktaları (Sarı->Turuncu->Kırmızı)
                plt.scatter(
                    df_disease['Konum_X'], 
                    df_disease['Konum_Y'], 
                    c=df_disease['Risk_Skoru(%)'], 
                    cmap='YlOrRd', 
                    s=df_disease['Risk_Skoru(%)'] * 2 + 20, # Riske göre büyüklük
                    edgecolor='darkred',
                    alpha=0.9, 
                    label='Hastalık (Sarı-Kırmızı Skala)'
                )
                
                # Hastalık isimlerini ve risk yüzdesini noktaların üstüne yazdır
                for idx, row in df_disease.iterrows():
                    plt.text(
                        row['Konum_X'] + 0.15, # Noktanın hemen sağına
                        row['Konum_Y'] + 0.15, # Noktanın hemen üstüne
                        f"{row['Hastalik_Turu']}\n(%{row['Risk_Skoru(%)']:.0f})", 
                        fontsize=8, 
                        color='maroon',
                        fontweight='bold',
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.2')
                    )

            # 2. SAĞLIKLI NOKTALAR (Yeşil Noktalar)
            if len(df_healthy) > 0:
                plt.scatter(
                    df_healthy['Konum_X'], 
                    df_healthy['Konum_Y'], 
                    c='forestgreen', 
                    s=60, 
                    marker='o',
                    edgecolor='white',
                    alpha=0.9, 
                    label='Sağlıklı Bitki'
                )
                # İstersen sağlıklı olanların da yanına "Healthy" yazdırabilirsin:
                for idx, row in df_healthy.iterrows():
                    plt.text(
                        row['Konum_X'] + 0.15, 
                        row['Konum_Y'] + 0.15, 
                        "Healthy", 
                        fontsize=7, 
                        color='forestgreen'
                    )

            # --- Ortak Grafik Ayarları ---
            plt.title('Robotanik Canlı Sera Haritası (Sağlık & Risk Analizi)', fontsize=14, pad=15)
            plt.xlabel('Sera X Koordinatı (m)')
            plt.ylabel('Sera Y Koordinatı (m)')
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.legend(loc='upper right')
                
            # Değişiklikleri canlı ekrana yansıt
            plt.draw()
            plt.pause(0.01) 
            
            # 3. ÇIKTILARI MASAÜSTÜNE KAYDET (PNG + PDF)
            plt.savefig(self.output_png_path, dpi=150, bbox_inches='tight')
            plt.savefig(self.output_pdf_path, format='pdf', bbox_inches='tight')
            
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
