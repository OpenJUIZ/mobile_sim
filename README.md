# mobile_sim

[JUIZ](https://github.com/OpenJUIZ/juiz) を使った平面移動ロボットのシミュレータです。白黒画像のマップの上でロボットを動かし、LiDAR をシミュレートし、ブラウザで様子を見られます。`.juizapp` として配布し、`juiz run` だけで動かせるようにします。

> 状態: 開発中です。シミュレーション本体と juiz のコンポーネントまでできています(ブラウザ画面と .juizapp の配布はまだ)。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/mobile_sim/issues) を見てください。

## できること(予定)

- JUIZ から速度(`juiz_std.geometry.Twist2D`)を受けてロボットを動かす。全方向移動と対向 2 輪を切り替えられる
- 白黒画像のマップと、原点・解像度を書いた mapconf(ROS の map_server と同じ YAML)を読み込む
- マップとの衝突判定
- マップ上での LiDAR のシミュレーション(`juiz_std.sensor.LaserScan`)
- ブラウザの画面で、マップ・ロボット・スキャンを表示し、キーボードで操作する
- Docker で実行する
- 後で GPS と IMU を追加する

[OpenJUIZ/juiz](https://github.com/OpenJUIZ/juiz) の `examples/python/turtle_sim_py` を Rust に移植したものです。

## 使い方(予定)

```sh
# juiz を入れる(配布する .juizapp と同じ版)
pip install juiz-cli

# Releases から取ってきたパッケージを動かす
juiz run mobile_sim-0.1.0.juizapp
# ブラウザで http://localhost:8000/ を開く
```

Docker なら次のとおりです。

```sh
docker run -p 8000:8000 ghcr.io/openjuiz/mobile_sim
```

mapconf の例:

```yaml
image: default.png        # mapconf からの相対パス
resolution: 0.05          # [m/px]
origin: [-5.0, -5.0, 0.0] # 画像の左下の画素の世界座標 [m, m, rad]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

## 構成(予定)

| パス | 中身 |
| --- | --- |
| `crates/mobile_sim_core/` | 車両モデル、マップ、衝突判定、LiDAR(JUIZ に依存しない) |
| `crates/mobile_sim/` | JUIZ のコンポーネント(cdylib) |
| `app/` | `juiz pack` の入力(`juizapp.yaml`、`app.conf`、マップ、ブラウザ画面) |
| `docker/` | Dockerfile と compose.yml |
| `examples/` | 速度を送るサンプル |

## 開発

```sh
cargo build --workspace   # target/debug/libmobile_sim.so などができる
cargo test --workspace
```

Rust 1.88 以降が要ります。juiz_sdk は `=0.0.29` に固定しています(Rust のプラグインは juiz と juiz_sdk の版が一致しないと読み込めないため)。

### juiz で動かす

`juiz` 0.0.29 を、プラグインと同じ rustc でビルドして入れます(`cargo install juiz_app --version 0.0.29 --locked`)。

```sh
scripts/dev_plugin.sh            # プラグインをビルドして app/plugins/<target>/ に置く
cd app && juiz -f app.conf -d    # http://localhost:8000/ で HTTP ブローカーが立つ
python3 scripts/integration_test.py   # 別の端末で。juiz を起動して HTTP から各プロセスを呼ぶ
```

コンテナ `mobile_sim`(インスタンス名 `sim0`)の ContainerProcess:

| ContainerProcess | 引数 | 戻り値 |
| --- | --- | --- |
| `mobile_sim_step` | なし | なし(main_loop_ec が 50 Hz で呼ぶ) |
| `mobile_sim_set_velocity` | `velocity: Twist2D`, `index` | なし。トピック `cmd_vel` を購読 |
| `mobile_sim_get_pose` | `index` | `Pose2DStamped` |
| `mobile_sim_get_odometry` | `index` | `{time, pose, twist, distance}` |
| `mobile_sim_get_scan` | `index` | `LaserScan`(何にも当たらないビームは `range_max + 1`) |
| `mobile_sim_get_map` | なし | 画像(PNG) |
| `mobile_sim_get_map_info` | なし | mapconf の内容と画像の大きさ |
| `mobile_sim_load_map` | `map`(mapconf のパス) | なし |
| `mobile_sim_reset_pose` | `pose: Pose2D`, `index` | なし |
| `mobile_sim_get_state` | なし | 全ロボットの姿勢・速度・スキャン(画面用) |

ロボットの形・速度の上限・LiDAR などは `app/app.conf` のコンテナの引数で変えられます(書き方は `crates/mobile_sim/src/config.rs`)。マップのパスは環境変数 `MOBILE_SIM_MAP` でも上書きできます。

## ライセンス

MIT または Apache-2.0 のどちらか(デュアルライセンス)です。[LICENSE-MIT](LICENSE-MIT)・[LICENSE-APACHE](LICENSE-APACHE) を見てください。
