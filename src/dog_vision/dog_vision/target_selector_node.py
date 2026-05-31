#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dog_bringup.msg import Detection2DArray

class TargetSelectorNode(Node):
    def __init__(self):
        super().__init__('target_selector_node')
        
        # 使用 ROS 参数，方便你在比赛时通过终端或 Launch 文件实时更改抓取目标
        # Row: 0为第一排(近)，1为第二排(远)
        # Col: 0~3，从左往右数
        self.declare_parameter('target_row', 0)
        self.declare_parameter('target_col', 1) 
        
        # 1. 订阅 YOLO 发出的纯视觉 2D 框
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/vision/detections',
            self.detection_callback,
            10)
            
        # 2. 发布逻辑 ID 给雷达定位节点
        self.publisher_ = self.create_publisher(
            Int32MultiArray, 
            '/vision/target_id', 
            10)
            
        self.get_logger().info('视觉逻辑决策节点已启动，正在监听 YOLO 数据...')
        self.last_published_target = None

    def detection_callback(self, msg):
        # 1. 容错处理：只要画面里有箱子（哪怕只有 1 个），系统就保持工作状态
        # 完全解除对箱子数量的限制！
        if len(msg.detections) == 0:
            return
            
        # 2. 读取当前需要抓取的目标参数
        target_row = self.get_parameter('target_row').value
        target_col = self.get_parameter('target_col').value
        
        # 3. 校验逻辑：放开限制，支持完整的 2排 4列（即 8 个箱子的满血状态）
        if target_row not in [0, 1] or target_col not in [0, 1, 2, 3]:
            self.get_logger().error(f'目标参数错误: row={target_row}, col={target_col}')
            return
            
        # 4. 直接发布指令给雷达节点
        # 雷达（C++）那边已经具备绝对网格定位能力，只要雷达格子里有箱子，就能匹配上！
        out_msg = Int32MultiArray()
        out_msg.data = [target_row, target_col]
        self.publisher_.publish(out_msg)
        
        # 5. 打印日志 (防刷屏)
        current_target = (target_row, target_col)
        if current_target != self.last_published_target:
            self.get_logger().info(f'抓取 第 {target_row + 1} 排，左起第 {target_col + 1} 个')
            self.last_published_target = current_target
            
def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()