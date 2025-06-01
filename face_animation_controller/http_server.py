#!/usr/bin/env python3

from flask import Flask, request, jsonify
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class FaceHttpServer(Node):
    """
    HTTP APIサーバーとROS2 TOPICの橋渡しを行うノード
    """

    def __init__(self):
        super().__init__('face_http_server')
        
        # パラメータの宣言
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('topic_name', '/face_expression')
        
        # パラメータの取得
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # 現在の表情を保持
        self.current_expression = "neutral"
        
        # 有効な表情のリスト
        self.valid_expressions = [
            'neutral', 'happy', 'angry', 'sad', 'surprised', 'crying', 'hurt'
        ]
        
        # ROS2 Publisher - 表情指示を送信
        self.expression_publisher = self.create_publisher(
            String,
            topic_name,
            10
        )
        
        # Flask アプリケーションのセットアップ
        self.app = Flask(__name__)
        CORS(self.app)  # CORSを有効化
        
        # ルートの設定
        self.setup_routes()
        
        self.get_logger().info(f'Face HTTP Server initialized')
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        self.get_logger().info(f'Server will start on {self.host}:{self.port}')

    def setup_routes(self):
        """
        Flask ルートの設定
        """
        
        @self.app.route('/expression', methods=['GET'])
        def get_expression():
            """現在の表情を取得"""
            return jsonify({"expression": self.current_expression})
        
        @self.app.route('/expression', methods=['POST'])
        def set_expression():
            """表情を設定"""
            try:
                data = request.get_json()
                
                if not data or 'expression' not in data:
                    return jsonify({"error": "Missing 'expression' field"}), 400
                
                expression = data['expression'].strip().lower()
                
                # 有効な表情かチェック
                if expression not in self.valid_expressions:
                    return jsonify({
                        "error": f"Invalid expression: {expression}",
                        "valid_expressions": self.valid_expressions
                    }), 400
                
                # 表情を更新
                old_expression = self.current_expression
                self.current_expression = expression
                
                # ROS2 TOPICに表情を発行
                msg = String()
                msg.data = expression
                self.expression_publisher.publish(msg)
                
                self.get_logger().info(f'Expression change: {old_expression} -> {expression}')
                
                return jsonify({
                    "success": True,
                    "expression": expression,
                    "previous_expression": old_expression
                }), 200
                
            except Exception as e:
                self.get_logger().error(f'Error processing request: {str(e)}')
                return jsonify({"error": str(e)}), 500
        
        @self.app.route('/status', methods=['GET'])
        def get_status():
            """サーバーの状態を取得"""
            return jsonify({
                "status": "running",
                "current_expression": self.current_expression,
                "valid_expressions": self.valid_expressions,
                "node_name": self.get_name()
            })
        
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """ヘルスチェック"""
            return jsonify({"status": "healthy"}), 200

    def run_server(self):
        """
        Flaskサーバーを実行
        """
        try:
            self.get_logger().info(f'Starting HTTP server on {self.host}:{self.port}')
            self.app.run(
                host=self.host,
                port=self.port,
                debug=False,
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start HTTP server: {str(e)}')


def ros_spin(node):
    """
    ROS2ノードを別スレッドで実行
    """
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'ROS2 spin error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # ノードを作成
        server_node = FaceHttpServer()
        
        # ROS2ノードを別スレッドで実行
        ros_thread = threading.Thread(target=ros_spin, args=(server_node,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # メインスレッドでFlaskサーバーを実行
        server_node.run_server()
        
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()