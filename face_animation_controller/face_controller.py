#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
from typing import Optional

class FaceAnimationController(Node):
    """
    ROS2 TOPICから表情指示を受け取り、HTTP APIを通じて顔アニメーションを制御するノード
    """

    def __init__(self):
        super().__init__('face_animation_controller')
        
        # パラメータの宣言
        self.declare_parameter('face_api_url', 'http://localhost:8080')
        self.declare_parameter('topic_name', '/face_expression')
        
        # パラメータの取得
        self.face_api_url = self.get_parameter('face_api_url').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # 現在の表情を保持
        self.current_expression = "neutral"
        
        # 有効な表情のリスト
        self.valid_expressions = [
            'neutral', 'happy', 'angry', 'sad', 'surprised', 'crying', 'hurt', 'wink'
        ]
        
        # Subscriberの作成
        self.expression_subscriber = self.create_subscription(
            String,
            topic_name,
            self.expression_callback,
            10
        )
        
        # HTTP接続の状態
        self.is_connected = False
        
        self.get_logger().info(f'Face Animation Controller started')
        self.get_logger().info(f'Listening to topic: {topic_name}')
        self.get_logger().info(f'Face API URL: {self.face_api_url}')
        self.get_logger().info(f'Valid expressions: {", ".join(self.valid_expressions)}')

    def expression_callback(self, msg: String):
        """
        表情指示を受け取るコールバック関数
        """
        requested_expression = msg.data.strip().lower()
        
        # 有効な表情かチェック
        if requested_expression not in self.valid_expressions:
            self.get_logger().warn(
                f'Invalid expression received: "{requested_expression}". '
                f'Valid expressions: {", ".join(self.valid_expressions)}'
            )
            return
        
        # 同じ表情の場合はスキップ
        if requested_expression == self.current_expression:
            self.get_logger().debug(f'Expression is already "{requested_expression}", skipping')
            return
        
        # 表情を更新
        old_expression = self.current_expression
        self.current_expression = requested_expression
        
        self.get_logger().info(
            f'Expression change requested: {old_expression} -> {requested_expression}'
        )
        
        # HTTP APIに表情を即座に送信
        self.send_expression_to_api(requested_expression)

    def send_expression_to_api(self, expression: str) -> bool:
        """
        HTTP APIに表情データを送信（0.1秒以内での応答を目指す）
        """
        try:
            url = f"{self.face_api_url}/expression"
            headers = {'Content-Type': 'application/json'}
            data = {'expression': expression}
            
            response = requests.post(
                url, 
                headers=headers, 
                json=data, 
                timeout=0.08  # 0.08秒でタイムアウト
            )
            
            if response.status_code == 200:
                self.get_logger().info(f'Successfully sent expression "{expression}" to API')
                self.is_connected = True
                return True
            else:
                self.get_logger().error(
                    f'Failed to send expression. HTTP {response.status_code}: {response.text}'
                )
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {str(e)}')
            self.is_connected = False
            return False

    def get_expression_from_api(self) -> Optional[str]:
        """
        HTTP APIから現在の表情を取得
        """
        try:
            url = f"{self.face_api_url}/expression"
            response = requests.get(url, timeout=0.08)
            
            if response.status_code == 200:
                data = response.json()
                if 'expression' in data:
                    self.is_connected = True
                    return data['expression']
                else:
                    self.get_logger().warn('Invalid response format from API')
                    return None
            else:
                self.get_logger().error(
                    f'Failed to get expression. HTTP {response.status_code}: {response.text}'
                )
                return None
                
        except requests.exceptions.RequestException as e:
            self.get_logger().debug(f'HTTP request failed: {str(e)}')
            self.is_connected = False
            return None

    def publish_status(self):
        """
        現在の状態を他のノードに通知するためのメソッド（将来の拡張用）
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        face_controller = FaceAnimationController()
        
        # ノードの実行
        rclpy.spin(face_controller)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()