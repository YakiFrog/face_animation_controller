#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

class ExpressionPublisher(Node):
    """
    表情指示をTOPICに送信するテスト用ノード
    """

    def __init__(self):
        super().__init__('expression_publisher')
        
        # パラメータの宣言
        self.declare_parameter('topic_name', '/face_expression')
        
        # パラメータの取得
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        
        # Publisher の作成
        self.publisher = self.create_publisher(String, topic_name, 10)
        
        # 有効な表情のリスト
        self.valid_expressions = [
            'neutral', 'happy', 'angry', 'sad', 'surprised', 'crying', 'hurt'
        ]
        
        self.get_logger().info(f'Expression Publisher started')
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        self.get_logger().info(f'Valid expressions: {", ".join(self.valid_expressions)}')
        self.get_logger().info('Commands:')
        self.get_logger().info('  1: neutral, 2: happy, 3: angry, 4: sad')
        self.get_logger().info('  5: surprised, 6: crying, 7: hurt')
        self.get_logger().info('  q: quit, a: auto demo')

    def publish_expression(self, expression: str):
        """
        指定された表情をTOPICに送信
        """
        if expression not in self.valid_expressions:
            self.get_logger().warn(f'Invalid expression: {expression}')
            return False
        
        msg = String()
        msg.data = expression
        self.publisher.publish(msg)
        self.get_logger().info(f'Published expression: {expression}')
        return True

    def run_interactive(self):
        """
        インタラクティブモードで実行
        """
        print("\n=== Expression Publisher Interactive Mode ===")
        print("Commands:")
        print("  1: neutral, 2: happy, 3: angry, 4: sad")
        print("  5: surprised, 6: crying, 7: hurt")
        print("  q: quit, a: auto demo")
        print("Enter command: ", end="", flush=True)
        
        while rclpy.ok():
            try:
                # 入力を待機
                command = input().strip().lower()
                
                if command == 'q':
                    break
                elif command == 'a':
                    self.run_auto_demo()
                elif command in '1234567':
                    expression_map = {
                        '1': 'neutral',
                        '2': 'happy', 
                        '3': 'angry',
                        '4': 'sad',
                        '5': 'surprised',
                        '6': 'crying',
                        '7': 'hurt'
                    }
                    self.publish_expression(expression_map[command])
                else:
                    print(f"Invalid command: {command}")
                
                print("Enter command: ", end="", flush=True)
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break

    def run_auto_demo(self):
        """
        自動デモモードで全ての表情を順番に実行
        """
        print("\n=== Auto Demo Mode ===")
        print("Press Ctrl+C to stop demo")
        
        try:
            for i, expression in enumerate(self.valid_expressions):
                if not rclpy.ok():
                    break
                
                print(f"Demo {i+1}/{len(self.valid_expressions)}: {expression}")
                self.publish_expression(expression)
                time.sleep(2.0)  # 2秒間隔
            
            print("Demo completed!")
            
        except KeyboardInterrupt:
            print("\nDemo stopped by user")

    def run_single_expression(self, expression: str):
        """
        単一の表情を送信して終了
        """
        if self.publish_expression(expression):
            self.get_logger().info(f'Single expression "{expression}" sent successfully')
        else:
            self.get_logger().error(f'Failed to send expression "{expression}"')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher_node = ExpressionPublisher()
        
        # コマンドライン引数をチェック
        if len(sys.argv) > 1:
            expression = sys.argv[1].lower()
            # 単一の表情を送信
            publisher_node.run_single_expression(expression)
        else:
            # インタラクティブモードで実行
            publisher_node.run_interactive()
        
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()