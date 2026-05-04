from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Kamera Düğümü
    camera_node = Node(
        package='robotanik_vision',      # Paketinin adı
        executable='camera_node',        # setup.py'de belirlediğin çalıştırılabilir ad
        name='camera_node',
        output='screen'                  # 'Kamera verisi yayınlanıyor...' logunu görmek için
    )

    # 2. AI Analiz Düğümü
    ai_analyzer_node = Node(
        package='robotanik_vision',      # Paketinin adı
        executable='ai_analyzer',        # setup.py'de belirlediğin çalıştırılabilir ad
        name='ai_analyzer',
        output='screen'                  # Olası hata loglarını terminalde görmek için
    )

    return LaunchDescription([
        camera_node,
        ai_analyzer_node
    ])
