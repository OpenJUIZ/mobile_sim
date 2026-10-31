# mobile_sim 計画書(turtle_sim の Rust 移植と juizapp 配布)

2026-10-31 作成

## 目的と要件

OpenJUIZ/juiz の examples にある Python 版 turtle_sim を Rust に移植し、平面移動ロボットのシミュレータとして `.juizapp` で配布する。要件は次の 6 つ。

| # | 要件 | 方針 |
| --- | --- | --- |
| 1 | 平面移動ロボット、JUIZ で速度を受け付ける | `juiz_std.geometry.Twist2D` を受ける ContainerProcess。トピック購読でも直接呼び出しでも受け付ける |
| 2 | マップを白黒画像から読む | PNG/PGM を `image` クレートで読み、閾値で占有格子にする |
| 3 | 原点・スケールなどを mapconf で読む | YAML。ROS の map_server と同じ項目(image, resolution, origin, occupied_thresh, free_thresh, negate)を推奨([決めてほしいこと](#決めてほしいこととリスク)) |
| 4 | マップ上で LiDAR をシミュレート | 各ビームを格子上でレイキャストし `juiz_std.sensor.LaserScan` で返す |
| 5 | 画面はブラウザ | HTTP ブローカーの `static_filepaths` で HTML/JS を配信し、juiz.js でプロセスを呼ぶ |
| 6 | Docker でも実行 | `juiz_app` を入れたイメージで `juiz run mobile_sim-x.y.z.juizapp` を動かす |

リポジトリの説明にある GPS・IMU と衝突判定は、今回の要件には無いため後半のマイルストーンに置いた。

## 移植元 turtle_sim_py の中身

移植元は `examples/python/turtle_sim_py/`(Python 約 690 行)と、ブラウザ画面 `examples/turtle_sim/`(HTML + JS 約 520 行)の 2 か所。Python 側は 1 つの component に `turtle_sim` コンテナと 9 個のプロセスを持つ。

| プロセス | 引数 | 役割 |
| --- | --- | --- |
| `update` | なし | main_loop_ec(100 Hz)から呼ばれ、全タートルを前回からの実時間で積分 |
| `set_turtle_target_velocity` | index, velocity {vx, vy, wz} | 目標速度を設定。conf で `target_velocity0` を購読 |
| `get_turtle_pose` | index | {x, y, th} を返す。`turtle_pose0` に配信 |
| `get_turtle_lidar` | index | 200 ビーム、-1.5〜+1.5 rad、最大 10 m |
| `get_profile` | なし | 全タートルの姿勢・速度・加速度・LiDAR をまとめて返す(画面が使う) |
| `load_map` / `get_map` / `get_map_metadata` | map_metadata | 画像の読み込みと、画像・メタ情報の取得 |
| `spawn_turtle` | init_pose | タートルを追加 |

挙動として引き継ぐ点と、移植で直す点は次のとおり。

- 運動モデル: 目標速度へ加速度上限(0.1, 0.1 m/s², 0.05 rad/s²)で近づけ、速度上限(1.0, 1.0 m/s, 0.5 rad/s)で切る。中点角で積分する全方向移動。このまま移す。
- マップ: OpenCV で読み、輝度 127 未満を障害物とする。座標は「幅・高さ [m] + 左上の位置」で表しており、mapconf(原点・スケール)はまだ無い。
- LiDAR: Bresenham で画素をたどる。斜めの距離を y 方向のスケールだけで掛けている、x/y 入れ替え時の戻し方が怪しい、など精度に問題がある。Rust では DDA のレイキャストに書き直す。
- 衝突判定は無い(壁をすり抜ける)。位置も ±20 m で切っているだけ。
- 時間: `update` 内で `time.time()` の差を使うため、EC の周期に依存し、再現性がない。Rust では固定 dt も選べるようにする。
- 画面: `juiz_js/juiz.js` を conf で `./js` から配信しているが、実際のファイルは `bindings/js/juiz.js` にあり、conf のままでは動かない可能性が高い。mobile_sim では juiz.js をパッケージに同梱する。

## juizapp 形式と JUIZ 側の前提

配布形式は juiz の [docs/spec/app-package.md](https://github.com/OpenJUIZ/juiz/blob/main/docs/spec/app-package.md)(format 1、2026-10-30 確定)に従う。`juiz pack` / `juiz run` / `juiz install` は juiz_app 0.0.29 に実装済み。mobile_sim に関わる点は次のとおり。

- 中身は zip。ルートに `juizapp.yaml`、`app.conf`、`plugins/<target triple>/` にネイティブプラグイン、`assets/` に画像などを置く。conf は `path: "plugins/{target}"` と書き、juiz がホストの triple に置き換える。
- 1 つのパッケージに Linux x86_64 / arm64、macOS、Windows のバイナリを並べられる。CI でターゲットごとにビルドしてまとめ、最後に `juiz pack` する。
- Rust プラグインを含むと、`juiz: "=0.0.29"` のように juiz_sdk の版が完全一致、かつ `rust.rustc` がホストの juiz をビルドした rustc と一致しなければ動かない(§9 段 3・6)。利用者が `cargo install juiz_app` したときの rustc と、配布物をビルドした rustc がずれると弾かれる。配布上いちばん大きな制約。
- `kind: app` で単体にするか、`kind: base` で常駐させて制御アプリを別プロセスから `ipc://mobile_sim` でつなぐかを選べる。シミュレータは「ロボットの代わり」なので `kind: base` を推奨する。実機用の基礎パッケージと同じインターフェースにしておけば、同じ制御アプリを実機とシミュレータで切り替えられる。
- 型は `juiz_std_types` の `Pose2D`・`Twist2D`・`LaserScan`・`Imu` がそのまま使える。型の指紋が言語をまたいで一致するので、Python や C++ の制御アプリからもつながる。

JUIZ 本体に 1 つ足りないものがある。`option.http_broker.static_filepaths` のパスはそのまま `ServeDir::new(path)` に渡され、カレントディレクトリからの相対で解決される(`juiz_core/src/brokers/http/http_router/mod.rs`)。`juiz run` はカレントディレクトリを変えないので、パッケージに入れた HTML/JS が配信できない。プラグインの path と同じく「conf の親ディレクトリからの相対」にする変更を juiz 側に入れる必要がある(小さな PR で済む見込み)。

## 設計

シミュレーションの計算は JUIZ に依存しない純粋な Rust クレートに分け、JUIZ のコンポーネントはそれを包む薄い層にする。計算部分を `cargo test` だけで検証でき、juiz_sdk の版上げの影響も薄い層に閉じる。

### リポジトリ構成

- `Cargo.toml`: ワークスペース。`juiz_sdk = "=0.0.29"`、`juiz_std_types` を版固定
- `crates/mobile_sim_core/`: 車両モデル、マップと mapconf、衝突判定、LiDAR のレイキャスト(JUIZ 非依存)
- `crates/mobile_sim/`: cdylib。`#[juiz_component_container]` と `#[juiz_component_container_process]` で 1 つの .so にまとめる
- `app/`: `juiz pack` の入力。`juizapp.yaml`、`app.conf`、`plugins/{target}/`(CI が置く)、`assets/maps/`(既定マップの PNG と YAML)、`web/`(index.html、juiz.js、描画用 JS)
- `docker/`: Dockerfile と compose.yml
- `examples/`: 速度を送るサンプル(Python のテレオペ、壁よけ)
- `.github/workflows/`: テスト、ターゲットごとのビルド、pack、リリース

### コンテナとプロセス

コンテナ `mobile_sim` がマップとロボットの列を持ち、次の ContainerProcess を生やす。ロボットは最初は 1 台を前提にし、`index` 引数(既定 0)で複数台に備える。

| プロセス | 入力 | 出力 | 使い方 |
| --- | --- | --- | --- |
| `step` | なし(dt は固定値か実時間) | なし | main_loop_ec で 50–100 Hz |
| `set_velocity` | `Twist2D`, index | なし | トピック `cmd_vel` を購読。一定時間(既定 0.5 s)更新が無ければ停止 |
| `get_pose` | index | `Pose2DStamped` | トピック `pose` に配信 |
| `get_odometry` | index | 速度と推定位置 | オドメトリ(後でノイズを足せるように分ける) |
| `get_scan` | index | `LaserScan` | トピック `scan` に配信 |
| `get_map` / `get_map_info` | なし | image / mapconf の内容 | 画面が起動時に 1 回呼ぶ |
| `load_map` | mapconf のパス | なし | 実行中のマップ切り替え |
| `reset_pose` | `Pose2D`, index | なし | 画面から初期位置を置き直す |
| `get_state` | なし | 全ロボットの姿勢・速度・スキャン・衝突中か | 画面が 10–20 Hz で取りに来る(turtle_sim の get_profile に当たる) |

ロボットの形(円の半径)、速度・加速度の上限、全方向か対向 2 輪か(対向 2 輪では linear.y を無視)、LiDAR の角度範囲・本数・最大距離・取り付け位置は、conf のコンテナの引数で指定する。

### mapconf

ROS の map_server と同じ YAML にする。既存のマップ(SLAM で作ったものなど)をそのまま使える。

```yaml
image: default.png        # mapconf からの相対パス
resolution: 0.05          # [m/px]
origin: [-5.0, -5.0, 0.0] # 画像の左下の画素の世界座標 [m, m, rad]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

黒が障害物、白が通れる。その中間は「不明」とし、衝突と LiDAR では障害物扱いにするかを設定で選ぶ。`origin` の角度は最初は 0 のみ対応。

### 衝突と LiDAR

- 衝突: ロボットを円とし、移動後の円内に占有画素があればそのステップの移動を打ち消して速度を 0 にし、`bumper` フラグを立てる。壁に沿って滑らせるのは後で。
- LiDAR: 各ビームを格子上の DDA(Amanatides–Woo)でたどり、距離を世界座標で返す。射程外は `range_max` より大きい値(LaserScan の慣例)。ガウスノイズを設定で足せる。周期は `step` と別にし、要求時に計算する。

### ブラウザ画面

HTTP ブローカー(既定ポート 8000)の `static_filepaths` で `/` に `web/` を配信する。画面は turtle_sim の JS を継いで、Canvas にマップ、ロボット、LiDAR の点、軌跡を描く。速度パッドとキーボード(WASD)で `set_velocity` を呼ぶ。マップ上のクリックで `reset_pose`。ビルドの要らない素の JS にし、パッケージにそのまま入れる。

### Docker

`rust` イメージで `cargo install juiz_app --version 0.0.29` とプラグインのビルドを同じ rustc で行い、`debian:bookworm-slim` に `juiz` と `.juizapp` だけを載せるマルチステージ。同じ rustc で作るので juizapp の rustc 一致の問題が起きない。`docker run -p 8000:8000 ghcr.io/openjuiz/mobile_sim` でブラウザから開ける。別のマップは `-v ./maps:/maps` と環境変数で渡す。

## 実装の進め方

6 段階に分け、各段階を 1 つの PR にする。M3 で「ブラウザで動かせる turtle_sim 相当」が揃い、M4 で初めて `.juizapp` を配れる。各段階は GitHub の issue で管理する。

1. **M0 雛形**: ワークスペース、README、ライセンス(juiz と同じ MIT OR Apache-2.0)、CI(fmt、clippy、test)。
2. **M1 シミュレーション本体**(`mobile_sim_core`): mapconf と画像の読み込み、車両モデル(turtle の加速度・速度上限を移植)、衝突判定、LiDAR。単体テストで、既知の形のマップでの距離、座標変換、壁で止まることを確かめる。
3. **M2 JUIZ コンポーネント**(`mobile_sim` cdylib): 設計のプロセス群と `app.conf`。`juiz -f app/app.conf` で起動し、HTTP で `set_velocity` を呼ぶと `get_pose` が変わることを結合テストにする。
4. **M3 ブラウザ画面**: turtle_sim の JS を移植・整理して `app/web/` に置く。Playwright で画面が表示され、キー操作でロボットが動くことを確かめる。並行して juiz 本体に `static_filepaths` を conf 相対にする PR を出す。
5. **M4 juizapp 化と配布**: `juizapp.yaml`(`kind: base`、`base.http_port: 8000`)、ターゲットごとのビルドと `juiz pack` を GitHub Actions で行い、タグで Releases に `.juizapp` を載せる。`juiz run mobile_sim-0.1.0.juizapp` が別の機械で動くことを確かめる。
6. **M5 Docker**: 設計の Dockerfile と compose.yml、ghcr.io への公開。

その後に、リポジトリの説明にある GPS(ノイズつき位置)と IMU(`Imu` 型)、オドメトリのノイズ、複数台、壁沿いの滑りを足す。

## 決めてほしいこととリスク

次の 4 点は推奨で進め、違う場合だけ指摘してほしい。

| 決めること | 推奨 | 理由 |
| --- | --- | --- |
| mapconf の形式 | ROS map_server の YAML | 既存のマップとツールがそのまま使える。turtle_sim の「幅・高さ・左上」形式は使わない |
| パッケージの種類 | `kind: base` | 制御アプリを別の juizapp として `ipc://mobile_sim` でつなげ、実機と入れ替えられる |
| 車両モデル | 全方向と対向 2 輪を conf で切り替え(既定は対向 2 輪) | 多くの移動ロボットが対向 2 輪。turtle_sim は全方向 |
| 速度の型 | `juiz_std.geometry.Twist2D` | 標準型で他言語と指紋が一致する。turtle_sim の {vx, vy, wz} は使わない |

既知のリスクは次の 3 つ。

- **rustc の一致**: Rust プラグイン入りの juizapp は、利用者の juiz と同じ rustc でビルドしていないと動かない。配布する rustc を、`pip install juiz-cli` の wheel をビルドした rustc に合わせるのが現実的。README に「対応する juiz の入れ方」を明記し、結局動かない人には Docker を案内する。
- **static_filepaths の相対パス**: juiz 本体の修正が入るまで、juizapp から画面を配信できない。M3 の前に juiz 側に PR を出す。
- **juiz_sdk の版上げ**: juiz は 0.0.x で版が変わるたびに juizapp を作り直す必要がある。juiz のリリースに合わせて CI で再ビルドする仕組みを M4 に含める。
