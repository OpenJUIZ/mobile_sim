# mobile_sim

[JUIZ](https://github.com/OpenJUIZ/juiz) を使った平面移動ロボットのシミュレータです。白黒画像のマップの上でロボットを動かし、LiDAR をシミュレートし、ブラウザで様子を見られます。`.juizapp` として配布し、`juiz run` だけで動かせるようにします。

> 状態: 開発中です。シミュレーション本体、juiz のコンポーネント、ブラウザ画面、.juizapp のビルドまでできています(Docker はまだ)。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/mobile_sim/issues) を見てください。

## できること(予定)

- JUIZ から速度(`juiz_std.geometry.Twist2D`)を受けてロボットを動かす。全方向移動と対向 2 輪を切り替えられる
- 白黒画像のマップと、原点・解像度を書いた mapconf(ROS の map_server と同じ YAML)を読み込む
- マップとの衝突判定
- マップ上での LiDAR のシミュレーション(`juiz_std.sensor.LaserScan`)
- ブラウザの画面で、マップ・ロボット・スキャンを表示し、キーボードで操作する
- Docker で実行する
- 後で GPS と IMU を追加する

[OpenJUIZ/juiz](https://github.com/OpenJUIZ/juiz) の `examples/python/turtle_sim_py` を Rust に移植したものです。

## 使い方

[Releases](https://github.com/OpenJUIZ/mobile_sim/releases) から `.juizapp` を取ってきて、同じ版の juiz で動かします。

```sh
# juiz を入れる。版はパッケージの juizapp.yaml の juiz(Releases の説明にも書きます)と同じにする
pip install juiz-cli==0.0.29

juiz run mobile_sim-0.1.0.juizapp
# ブラウザで http://localhost:8000/ を開く
```

- Linux(x86_64・arm64)、macOS(Intel・Apple Silicon)、Windows(x86_64)のプラグインが入っています。
- Rust のプラグインは、juiz と juiz_sdk の版も、juiz をビルドした rustc の版も一致しないと読み込めません。パッケージは pip の `juiz-cli` と同じ rustc でビルドしているので、`cargo install` で入れた juiz では動かないことがあります。
- 基礎パッケージ(`kind: base`)なので、別の juiz アプリから `depends: { mobile_sim: "^0.1" }` でつなげます(IPC の名前は `mobile_sim`)。
- juiz 0.0.29 では、`juiz run` で起動するとブラウザ画面が配信されません(静的ファイルのパスをカレントディレクトリから探すため。OpenJUIZ/juiz#318 で直り、次の版から効きます)。HTTP の API は動きます。

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

### パッケージを作る

CI の Release ワークフローが、ターゲットごとにプラグインをビルドして `.juizapp` にまとめ、`juiz run` で結合テストをします。タグ `v<版>`(`app/juizapp.yaml` の `version` と同じ)を push すると Releases に載ります。手元では次のとおりです。

```sh
python3 scripts/juiz_rustc.py 0.0.29          # pip の juiz-cli をビルドした rustc の版(1.99.1)
rustup toolchain install 1.99.1
cargo +1.99.1 build -p mobile_sim --release
mkdir -p /tmp/plugins/x86_64-unknown-linux-gnu && cp target/release/libmobile_sim.so /tmp/plugins/x86_64-unknown-linux-gnu/
scripts/pack.sh /tmp/plugins dist            # dist/mobile_sim-0.1.0.juizapp
python3 scripts/integration_test.py --juizapp dist/mobile_sim-0.1.0.juizapp
```

juiz の新しい版に合わせるときは、`Cargo.toml` の `juiz_sdk`・`juiz_std_types` と `app/juizapp.yaml` の `juiz` を上げます。rustc の版は CI が juiz-cli の wheel から読みます。

### ブラウザ画面

juiz を起動して http://localhost:8000/ を開きます。画面は `app/web/` にあり、HTTP ブローカーの `static_filepaths` で配信します。

- WASD か矢印キー、または右のパッドをドラッグして動かす(離すと止まる)
- マップをクリックするとそこへロボットを置き、ドラッグで向きを決める
- スキャンの線と軌跡は切り替えられる

juiz 0.0.29 は `static_filepaths` を起動した場所から探すので、`app/` で起動してください(conf の場所基準にする修正は OpenJUIZ/juiz#318)。画面のテストは `node scripts/ui_test.js`(Playwright が要ります。juiz を `app/` で起動してから)。

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
