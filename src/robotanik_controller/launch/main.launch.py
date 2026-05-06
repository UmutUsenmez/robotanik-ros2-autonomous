import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Paket Yolları (Kendi paket isimlerine göre kontrol et)
    sim_pkg_dir = get_package_share_directory('robotanik_sim')
    vision_pkg_dir = get_package_share_directory('robotanik_vision')
    
    # DÜZELTME 1: Doğru paket ismini verdik
    controller_pkg_name = 'robotanik_controller' 

    # 2. Simülasyon Launch'ını Dahil Etme
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, 'launch', 'sim.launch.py') 
        )
    )

    # 3. Vision Launch'ını Dahil Etme (Gazebo'nun açılmasına zaman tanımak için 5sn bekliyor)
    vision_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # NOT: Eğer dosyanın adı "vision.launch.py" ise burayı ona göre geri değiştir. 
                # Şu an "vision_subsystem.launch.py" arayacak şekilde ayarlı.
                os.path.join(vision_pkg_dir, 'launch', 'vision_subsystem.launch.py')
            )
        )]
    )

    # 4. Main Controller (Sistem Beyni) Düğümü
    main_controller_node = TimerAction(
        period=8.0,
        actions=[Node(
            package=controller_pkg_name,     # Artık robotanik_controller paketine bakacak
            executable='main_controller',    
            name='main_controller_node',
            output='screen'
        )]
    )

    # 5. Canlı Heatmap (Isı Haritası) Düğümü
    # Main Controller'dan 2 saniye sonra (toplam 10. saniyede) devreye girer
    heatmap_node = TimerAction(
        period=10.0,
        actions=[Node(
            package=controller_pkg_name,
            executable='heatmap_node',
            name='heatmap_node',
            output='screen'
        )]
    )

    return LaunchDescription([
        sim_launch,
        vision_launch, # DÜZELTME 2: Değişken adını doğru yazdık
        main_controller_node,
        heatmap_node   # YENİ EKLENEN KISIM
    ])
