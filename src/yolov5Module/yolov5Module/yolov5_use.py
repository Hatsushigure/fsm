# 参考网址https://pypi.org/project/yolov5/ 
import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from self_interface.msg import YOLODetectResult
from yolov5 import YOLOv5
import cv2
import os
import torch
ros_distribution = os.environ.get("ROS_DISTRO") # 获得ros版本
print(f"device:{torch.device("cuda:0"if torch.cuda.is_available()else"cpu")}")

class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        # Declare ROS parameters.
        self.declare_parameter("device", "cuda:0", ParameterDescriptor(
            name="device", description="Compute device selection, default: cpu, options: cuda:0"))

        self.declare_parameter("model", "yolov5s", ParameterDescriptor(
            name="model", description="Default model selection: yolov5s"))
        
        self.declare_parameter("image_topic", "/rgb_camera/image_raw", ParameterDescriptor(
            name="image_topic", description="Image topic, default: /rgb_camera/image_raw"))
        
        self.declare_parameter("show_result", False, ParameterDescriptor(
            name="show_result", description="Whether to display detection results, default: False"))
       
        # 模型的加载
        model_path = "/home/fsm_test/src/yolov5Module/config" + self.get_parameter('model').value + ".pt"
        device = self.get_parameter('device').value
        self.yolov5 = YOLOv5(model_path=model_path, device=device)
        qos_profile = QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,  # 设置消息历史记录类型
            depth=10  # 设置消息历史记录深度
        )

        # 创建一个发布话题
        self.yolo_result_pub = self.create_publisher(
            YOLODetectResult, "yolo_result", 10)
        self.result_msg = YOLODetectResult()    # 要发送的消息存放的地方
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10, qos_profile=qos_profile)
        
        self.bridge = CvBridge()


    def image_callback(self, msg: Image):
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        detect_result = self.yolov5.predict(cv_image)
        self.get_logger().info(str(detect_result))  # 看看detect_result里面有什么

        # 解析结果
        predictions = detect_result.pred[0]
        # boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]
        score_array = []
        class_array = []
        # 这里面只需要获得每个结果的score和name就行
        for index in range(len(categories)):
            class_name = detect_result.names[int(categories[index])]
            score = float(scores[index])
            class_array.append(class_name)
            score_array.append(score)
        self.result_msg.names = class_array
        self.result_msg.scores = score_array
        self.result_msg.header.frame_id = "RGBcamera"
        if len(categories) > 0:
            # 获得类别就发送除去
            self.yolo_result_pub.publish(self.result_msg)

def main():
    rclpy.init()
    rclpy.spin(YoloV5Ros2())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
