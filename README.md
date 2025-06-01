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