# ROS 2 LiDAR Benchmark Package

Jetson Orin Nano SuperでLiDARデータのパフォーマンスを自動計測・可視化するROS 2パッケージ

## 機能

- **点群データ受信**: tcpreplayから送信される点群データをROS 2トピックとして処理
- **メトリクス計測**:
  - 公開頻度（Hz）
  - 帯域幅使用量（Mbps）
  - ジッター（ms）
- **システムリソース監視**:
  - CPU使用率
  - メモリ使用率
  - 温度（対応デバイスのみ）
- **リアルタイム可視化**: matplotlib使用のダッシュボード
- **自動分析レポート**: JSON形式の詳細レポート生成

## 必要な環境

- ROS 2 (Humble/Iron/Rolling)
- Python 3.8+
- 必要なPythonパッケージ:
  - psutil
  - matplotlib
  - numpy

## インストール

```bash
# ワークスペースのセットアップ
cd ~/ros2_ws/src
git clone <このリポジトリのURL>
cd ~/ros2_ws

# 依存関係のインストール
sudo apt update
sudo apt install python3-psutil python3-matplotlib python3-numpy

# ビルド
colcon build --packages-select ros2_lidar_benchmark
source install/setup.bash
```

## 使用方法

### 1. tcpreplayでLiDARデータを送信（別ターミナル）

```bash
# 例: pcapファイルから点群データを送信
sudo tcpreplay -i eth0 -l 0 lidar_data.pcap
```

### 2. ベンチマークの実行

#### 可視化ありの場合（デフォルト）:
```bash
ros2 launch ros2_lidar_benchmark benchmark.launch.py \
  input_topic:=/lidar/points \
  analysis_duration:=60.0
```

#### ヘッドレスモード（可視化なし）:
```bash
ros2 launch ros2_lidar_benchmark benchmark_headless.launch.py \
  input_topic:=/lidar/points \
  analysis_duration:=60.0 \
  report_file:=/tmp/benchmark_report.json
```

### 3. 個別ノードの実行

```bash
# 点群受信ノード
ros2 run ros2_lidar_benchmark pointcloud_receiver.py

# メトリクス計測ノード
ros2 run ros2_lidar_benchmark metrics_collector.py

# システムモニター
ros2 run ros2_lidar_benchmark system_monitor.py

# 可視化ノード
ros2 run ros2_lidar_benchmark visualizer.py

# 分析ノード（60秒間記録）
ros2 run ros2_lidar_benchmark benchmark_analyzer.py
```

## パラメータ

### Launch引数
- `input_topic`: 入力点群トピック（デフォルト: `/lidar/points`）
- `output_topic`: ベンチマーク用出力トピック（デフォルト: `/benchmark/points`）
- `analysis_duration`: 分析時間（秒）（デフォルト: 60.0）
- `enable_visualization`: 可視化の有効/無効（デフォルト: true）

### ノードパラメータ
各ノードの詳細パラメータはlaunchファイルを参照

## 出力

### レポートファイル
`/tmp/lidar_benchmark_report.json` に以下の情報を含むレポートが生成されます:

- 周波数統計（平均、最小、最大、標準偏差）
- ジッター統計
- 帯域幅使用量
- システムリソース使用率
- パフォーマンス評価
- 推奨事項

### 可視化
リアルタイムで以下の4つのグラフを表示:
- 公開頻度（Hz）
- ジッター（ms）
- 帯域幅使用量（Mbps）
- CPU/メモリ使用率（%）

## トラブルシューティング

### 点群データが受信されない場合
- tcpreplayが正しく実行されているか確認
- トピック名が正しいか確認: `ros2 topic list`
- データが流れているか確認: `ros2 topic hz /lidar/points`

### 可視化が表示されない場合
- X11フォワーディングが有効か確認（SSH接続の場合）
- `DISPLAY`環境変数が設定されているか確認
- ヘッドレスモードの使用を検討

### メモリ不足エラー
- `window_size`パラメータを小さくする
- 分析時間を短くする