# Face Animation Controller

ROS2 TOPICから表情指示を受け取り、HTTP APIを通じてSirius Face Animationアプリケーションを制御するROS2パッケージです。

## 概要

このパッケージは3つのメインコンポーネントで構成されています：

1. **face_http_server**: HTTP APIサーバーとROS2 TOPICの橋渡し
2. **face_controller**: ROS2 TOPICを受信してHTTP APIに転送
3. **expression_publisher**: テスト用の表情指示パブリッシャー

## システム構成

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ ROS2 Publishers │────→│ face_http_server│────→│ Sirius Face App │
│ (Other Nodes)   │    │                 │    │ (HTTP Client)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌─────────────────┐
                       │ face_controller │
                       │ (Optional)      │
                       └─────────────────┘
```

## インストール・ビルド

```bash
# ROS2ワークスペースに移動
cd ~/ros2_ws

# パッケージをビルド
colcon build --packages-select face_animation_controller

# 環境を読み込み
source install/setup.bash
```

## 使用方法

### 1. システム全体を起動

```bash
# 基本起動（HTTPサーバー + コントローラー）
ros2 launch face_animation_controller face_animation_system.launch.py

# カスタムポートで起動
ros2 launch face_animation_controller face_animation_system.launch.py port:=9090

# カスタムトピック名で起動
ros2 launch face_animation_controller face_animation_system.launch.py topic_name:=/custom_face_expression
```

### 2. 個別ノードの起動

#### HTTPサーバーのみ起動
```bash
ros2 run face_animation_controller face_http_server
```

#### フェイスコントローラーのみ起動
```bash
ros2 run face_animation_controller face_controller
```

### 3. 表情の送信

#### TOPICから送信
```bash
# 直接TOPICに送信
ros2 topic pub /face_expression std_msgs/String "data: 'happy'" --once

# インタラクティブなパブリッシャーを使用
ros2 run face_animation_controller expression_publisher

# 単一の表情を送信
ros2 run face_animation_controller expression_publisher happy
```

#### HTTP APIから送信
```bash
# curlコマンドで送信
curl -X POST http://localhost:8080/expression \
  -H "Content-Type: application/json" \
  -d '{"expression": "happy"}'

# 現在の表情を取得
curl http://localhost:8080/expression
```

## パラメータ設定

### face_http_server

| パラメータ | デフォルト値 | 説明 |
|------------|-------------|------|
| `port` | 8080 | HTTPサーバーのポート番号 |
| `host` | 0.0.0.0 | HTTPサーバーのホスト |
| `topic_name` | /face_expression | 監視するROS2トピック名 |

### face_controller

| パラメータ | デフォルト値 | 説明 |
|------------|-------------|------|
| `face_api_url` | http://localhost:8080 | Sirius Face AnimationのHTTP API URL |
| `topic_name` | /face_expression | 監視するROS2トピック名 |
| `update_rate` | 10.0 | 接続確認の更新レート（Hz） |

### 設定例

```bash
# カスタムパラメータで起動
ros2 run face_animation_controller face_http_server --ros-args \
  -p port:=9090 \
  -p host:=127.0.0.1 \
  -p topic_name:=/robot/face_expression

ros2 run face_animation_controller face_controller --ros-args \
  -p face_api_url:=http://192.168.1.100:8080 \
  -p topic_name:=/robot/face_expression \
  -p update_rate:=5.0
```

## 有効な表情

以下の7種類の表情がサポートされています：

- `neutral` - 通常の表情
- `happy` - 笑顔
- `angry` - 怒り
- `sad` - 悲しみ
- `surprised` - 驚き
- `crying` - 泣き
- `hurt` - 痛がる表情

## ROS2トピック仕様

### 基本トピック情報

| 項目 | 値 |
|------|-----|
| **トピック名** | `/face_expression` (デフォルト) |
| **メッセージタイプ** | `std_msgs/String` |
| **QoS** | Depth 10 |

### トピックにメッセージを送信する方法

#### 1. コマンドラインから送信

```bash
# 基本的な送信方法
ros2 topic pub /face_expression std_msgs/String "data: 'happy'" --once

# 継続的に送信（1Hzで繰り返し）
ros2 topic pub /face_expression std_msgs/String "data: 'angry'" --rate 1

# 各表情の送信例
ros2 topic pub /face_expression std_msgs/String "data: 'neutral'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'happy'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'angry'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'sad'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'surprised'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'crying'" --once
ros2 topic pub /face_expression std_msgs/String "data: 'hurt'" --once
```

#### 2. Pythonノードから送信

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FaceExpressionNode(Node):
    def __init__(self):
        super().__init__('face_expression_node')
        
        # パブリッシャーを作成
        self.publisher = self.create_publisher(
            String, 
            '/face_expression',  # トピック名
            10                   # QoS depth
        )
    
    def change_expression(self, expression: str):
        """表情を変更する"""
        msg = String()
        msg.data = expression  # 表情名を文字列として設定
        self.publisher.publish(msg)
        self.get_logger().info(f'表情を{expression}に変更しました')

# 使用例
def main():
    rclpy.init()
    node = FaceExpressionNode()
    
    # 表情を順番に変更
    expressions = ['happy', 'angry', 'sad', 'surprised', 'crying', 'hurt', 'neutral']
    for expr in expressions:
        node.change_expression(expr)
        time.sleep(2)  # 2秒待機
    
    rclpy.shutdown()
```

#### 3. C++ノードから送信

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class FaceExpressionNode : public rclcpp::Node
{
public:
    FaceExpressionNode() : Node("face_expression_node")
    {
        // パブリッシャーを作成
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/face_expression", 10);
    }
    
    void change_expression(const std::string& expression)
    {
        auto message = std_msgs::msg::String();
        message.data = expression;  // 表情名を設定
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "表情を%sに変更しました", expression.c_str());
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

// 使用例
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceExpressionNode>();
    
    // 表情を変更
    node->change_expression("happy");
    
    rclcpp::shutdown();
    return 0;
}
```

### メッセージフォーマット

```yaml
# std_msgs/String メッセージの構造
data: string    # 表情名（文字列）
```

**有効な表情名（data フィールドに設定可能な値）:**
- `"neutral"`
- `"happy"`  
- `"angry"`
- `"sad"`
- `"surprised"`
- `"crying"`
- `"hurt"`

### トピック監視・デバッグ

```bash
# トピックが存在するか確認
ros2 topic list | grep face_expression

# トピックの詳細情報を表示
ros2 topic info /face_expression

# トピックの型情報を表示
ros2 topic type /face_expression

# リアルタイムでメッセージを監視
ros2 topic echo /face_expression

# トピックの送信頻度を確認
ros2 topic hz /face_expression

# トピックの帯域幅を確認
ros2 topic bw /face_expression
```

### カスタムトピック名の使用

デフォルトのトピック名（`/face_expression`）以外を使用する場合：

```bash
# カスタムトピック名でシステム起動
ros2 launch face_animation_controller face_animation_system.launch.py topic_name:=/robot/facial_expression

# カスタムトピックに送信
ros2 topic pub /robot/facial_expression std_msgs/String "data: 'happy'" --once

# 個別ノードでカスタムトピック名を使用
ros2 run face_animation_controller face_http_server --ros-args -p topic_name:=/robot/facial_expression
ros2 run face_animation_controller face_controller --ros-args -p topic_name:=/robot/facial_expression
```

### 統合例：センサーと連携した表情制御

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class EmotionalRobotNode(Node):
    """センサーデータに基づいて表情を制御するロボットノード"""
    
    def __init__(self):
        super().__init__('emotional_robot_node')
        
        # 表情制御用パブリッシャー
        self.face_publisher = self.create_publisher(String, '/face_expression', 10)
        
        # センサーデータ用サブスクライバー
        self.distance_subscription = self.create_subscription(
            Range, '/ultrasonic_sensor', self.distance_callback, 10)
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.velocity_callback, 10)
        
        self.current_expression = 'neutral'
    
    def distance_callback(self, msg):
        """距離センサーの値に応じて表情を変更"""
        distance = msg.range
        
        if distance < 0.2:  # 20cm以内 - 驚き
            self.set_expression('surprised')
        elif distance < 0.5:  # 50cm以内 - 幸せ
            self.set_expression('happy')
        elif distance > 2.0:  # 2m以上 - 悲しい
            self.set_expression('sad')
        else:  # 通常
            self.set_expression('neutral')
    
    def velocity_callback(self, msg):
        """移動速度に応じて表情を変更"""
        speed = abs(msg.linear.x) + abs(msg.angular.z)
        
        if speed > 1.0:  # 高速移動 - 驚き
            self.set_expression('surprised')
        elif speed > 0.5:  # 通常移動 - 幸せ
            self.set_expression('happy')
    
    def set_expression(self, expression):
        """表情を設定（重複送信を避ける）"""
        if expression != self.current_expression:
            msg = String()
            msg.data = expression
            self.face_publisher.publish(msg)
            self.current_expression = expression
            self.get_logger().info(f'表情を{expression}に変更')

def main():
    rclpy.init()
    node = EmotionalRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### エラーハンドリング

```python
def safe_expression_publish(self, expression: str):
    """安全な表情送信（バリデーション付き）"""
    valid_expressions = ['neutral', 'happy', 'angry', 'sad', 'surprised', 'crying', 'hurt']
    
    if expression.lower() not in valid_expressions:
        self.get_logger().warn(f'無効な表情: {expression}. 有効な表情: {valid_expressions}')
        return False
    
    try:
        msg = String()
        msg.data = expression.lower()
        self.face_publisher.publish(msg)
        self.get_logger().info(f'表情を{expression}に変更しました')
        return True
    except Exception as e:
        self.get_logger().error(f'表情送信エラー: {str(e)}')
        return False
```

## HTTP API仕様

### エンドポイント

#### GET /expression
現在の表情を取得

**レスポンス:**
```json
{
  "expression": "happy"
}
```

#### POST /expression
表情を設定

**リクエスト:**
```json
{
  "expression": "angry"
}
```

**レスポンス:**
```json
{
  "success": true,
  "expression": "angry",
  "previous_expression": "happy"
}
```

#### GET /status
サーバーの状態を取得

**レスポンス:**
```json
{
  "status": "running",
  "current_expression": "neutral",
  "valid_expressions": ["neutral", "happy", "angry", "sad", "surprised", "crying", "hurt"],
  "node_name": "face_http_server"
}
```

#### GET /health
ヘルスチェック

**レスポンス:**
```json
{
  "status": "healthy"
}
```

## 実用例

### 1. 基本的なワークフロー

```bash
# ターミナル1: システム起動
ros2 launch face_animation_controller face_animation_system.launch.py

# ターミナル2: Sirius Face Animationアプリケーション起動
cd ~/sirius_face_anim
npm run dev

# ターミナル3: 表情テスト
ros2 run face_animation_controller expression_publisher
```

### 2. 他のROS2ノードからの制御

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.face_publisher = self.create_publisher(String, '/face_expression', 10)
    
    def change_expression(self, expression):
        msg = String()
        msg.data = expression
        self.face_publisher.publish(msg)
        self.get_logger().info(f'Changed face to: {expression}')

# 使用例
node = MyRobotNode()
node.change_expression('happy')  # 笑顔に変更
```

### 3. センサーデータとの連携

```python
# 例: 距離センサーの値に応じて表情を変更
def distance_callback(self, msg):
    distance = msg.range
    
    if distance < 0.3:  # 30cm以内
        self.change_expression('surprised')
    elif distance < 1.0:  # 1m以内
        self.change_expression('happy')
    else:
        self.change_expression('neutral')
```

## トラブルシューティング

### よくある問題

1. **HTTP接続エラー**
   ```bash
   # Sirius Face Animationアプリケーションが起動しているか確認
   curl http://localhost:8080/health
   
   # ポート使用状況確認
   netstat -tlnp | grep :8080
   ```

2. **ROS2トピックが見つからない**
   ```bash
   # トピックリストを確認
   ros2 topic list
   
   # トピックの詳細を確認
   ros2 topic info /face_expression
   ```

3. **パッケージが見つからない**
   ```bash
   # 環境設定を再読み込み
   source ~/ros2_ws/install/setup.bash
   
   # パッケージを再ビルド
   cd ~/ros2_ws
   colcon build --packages-select face_animation_controller
   ```

### ログ確認

```bash
# ノードのログを確認
ros2 node list
ros2 node info /face_http_server
ros2 node info /face_controller

# トピックをモニター
ros2 topic echo /face_expression
```

## 依存関係

- ROS2 (humble推奨)
- Python 3.8+
- requests
- flask
- flask-cors
- std_msgs

## ライセンス

MIT License