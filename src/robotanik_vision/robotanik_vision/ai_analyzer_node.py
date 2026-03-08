import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge
import cv2
import json # YENİ: Verileri paketlemek için
from ultralytics import YOLO

class AIAnalyzerNode(Node):
    def __init__(self):
        super().__init__('ai_analyzer_node')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.ai_publisher = self.create_publisher(String, 'ai/detections', 10)
        self.bridge = CvBridge()
        
        leaf_model_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_vision/models/leafdetectionfinal.pt'
        disease_model_path = '/home/aziz/Desktop/ros2_ws/src/robotanik_vision/models/yolo11s_leaf_disease.pt'
        
        self.leaf_model = YOLO(leaf_model_path)
        self.disease_model = YOLO(disease_model_path)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        annotated_frame = frame.copy()
        
        leaf_results = self.leaf_model(frame, conf=0.5, verbose=False)
        
        if len(leaf_results[0].boxes) > 0:
            disease_results = self.disease_model(frame, conf=0.45, verbose=False)
            disease_boxes = disease_results[0].boxes
            disease_names = disease_results[0].names

            for leaf_box in leaf_results[0].boxes:
                lx1, ly1, lx2, ly2 = map(int, leaf_box.xyxy[0])
                final_label = "Healthy"
                box_color = (0, 255, 0) 

                for d_box in disease_boxes:
                    dx1, dy1, dx2, dy2 = map(int, d_box.xyxy[0])
                    dcx = (dx1 + dx2) / 2
                    dcy = (dy1 + dy2) / 2
                    
                    if lx1 <= dcx <= lx2 and ly1 <= dcy <= ly2:
                        cls_id = int(d_box.cls[0])
                        final_label = disease_names[cls_id] 
                        box_color = (0, 0, 255) 
                        break 
                        
                cv2.rectangle(annotated_frame, (lx1, ly1), (lx2, ly2), box_color, 2)
                cv2.putText(annotated_frame, final_label, (lx1, ly1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                current_time = self.get_clock().now().nanoseconds / 1e9
                detection_data = {
                    "time": current_time,
                    "label": final_label
                }
                
                detection_msg = String()
                detection_msg.data = json.dumps(detection_data) # Veriyi metne çevirip fırlat
                self.ai_publisher.publish(detection_msg)

        cv2.imshow("Robotanik - Yapay Zeka Analizi", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AIAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
