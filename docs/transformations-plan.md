# Transformations 導入計画(mobile_sim・mapper_gmapping)

2026-11-03 作成。juiz 0.2.0 の Transformations(Container の位置姿勢、[仕様](https://github.com/OpenJUIZ/juiz/blob/main/docs/spec/transformations.md))を mobile_sim と mapper_gmapping で使う。

## 結論

- mobile_sim のロボットと LiDAR を 1 台ずつ Container に分け、真の位置とオドメトリを ContainerPose として書く。
- LiDAR の取付位置は conf の `transforms:`(`robot0 → lidar0`)だけに書く。
- mapper_gmapping は推定姿勢を自分の ContainerPose(`map → mapper0`)として書く。
- Transformations は 1 つの System の中で閉じている(システムをまたぐ Transform は juiz #376、将来)。mapper は mobile_sim と別の System なので、2 段階で進める。
  1. **段階 1**(juiz 0.2.0 で可能): 各 System の中で Transformations を使い、System の間はトピックでつなぐ。LiDAR の取付位置は mobile_sim がトピック `lidar_mount` で配る。
  2. **段階 2**(juiz #376 の後): mapper の `map` から mobile_sim の `odom0` へ Transform を張り、`map → odom → robot → lidar` を 1 本の木にする。

## juiz の制約で設計に効くもの

- Container そのものが座標系。`header.frame_id` には Container 名を入れる。
- ContainerProcess は**自分の** ContainerPose だけを `c.pose("基準")` / `c.set_pose("基準", p)` で読み書きする。`set_pose` は親 → 自分の Transform を書き換える。
- 1 つの Container が複数の Anchor の木に入れる(Anchor ごとに届く範囲が木であればよい)。
- Transform は自動では作られない。conf の `transforms:` に書く。
- Anchor(`static_anchor`・`periodic_anchor`)は ContainerProcess を持てない。

## 設計(段階 1)

```text
mobile_sim の System                       mapper_gmapping の System
  world (static_anchor)                      map (static_anchor)
    └─ robot0 (mobile_robot)  真の位置         └─ mapper0 (gmapping)  推定姿勢
         └─ lidar0 (mobile_lidar)  取付位置
  odom0 (static_anchor)
    └─ robot0                 オドメトリ
  トピック odom・scan・lidar_mount ─────────▶ mapper0
```

| Container(type) | 役割 | 主な ContainerProcess |
| --- | --- | --- |
| `world`(`static_anchor`) | 真の座標系。地図画像の座標と同じ | なし |
| `odom0`(`static_anchor`) | ロボット 0 のオドメトリの原点 | なし |
| `sim0`(`mobile_sim`) | 地図・時計・衝突・全ロボットの運動(mobile_sim_core のまま) | `step`、地図の Process、`get_state` |
| `robot0`(`mobile_robot`) | 台車 1 台。作られたときに `sim: sim0` の世界へ自分を登録する | `update`(`set_pose("world", 真)`・`set_pose("odom0", odom)`、戻り値をトピック `odom` へ)、`set_velocity`、`reset_pose` |
| `lidar0`(`mobile_lidar`) | LiDAR 1 台。取付位置は Transform `robot0 → lidar0` だけが持つ | `scan`(`pose("world")` で自分の位置を読み、地図に光線を飛ばす。frame_id は `lidar0`)、`mount`(`pose("robot0")` をトピック `lidar_mount` へ) |

- `robot0`・`lidar0` は、同じプラグインの中の表(名前 → 世界)で `sim0` の世界を共有する。ロックの順は「Container → 世界」に固定する。
- 1 周の順番(main_ec、20 Hz): `sim0.step` → `robot0.update` → `lidar0.scan`。Anchor は static なので、`update` が戻った時点で `lidar0` の位置は新しい。
- 今の `robots:` と `index` つきの Process は 0.3 系では残し、0.4 で消す。

mapper_gmapping は `map`(`static_anchor`)と Transform `map → mapper0` を置き、スキャンを処理するたびに最良の粒子の位置を `juiz::set_pose("map", ...)` で書く。LiDAR の取付位置は新しい Process `gmapping_set_laser_pose` がトピック `lidar_mount`(`PoseStamped`)を受けて設定する(受け取るまで初期化を待ち、変わったら作り直す。`laser_pose` は既定値として残す)。

## マイルストーン

| 番号 | リポジトリ | 中身 | 待つもの |
| --- | --- | --- | --- |
| T0 | mobile_sim | juiz 0.2.0 に上げる。juizapp の `juiz` を `>=0.2.0` に、rustc を揃える仕組みを外す | なし |
| T1 | mobile_sim | `mobile_robot`・`mobile_lidar` と世界の共有表。単体テスト | T0 |
| T2 | mobile_sim | app.conf・EC・トピック(`scan` の frame_id を `lidar0` に、`lidar_mount` を足す)。結合テストで `container/pose`・`transformations/tree` を確かめる | T1 |
| T3 | mobile_sim | 画面で `transformations/tree` からロボット・LiDAR・odom0 を描く | T2 |
| T4 | mobile_sim | 0.3.0 のリリース | T3 |
| G1 | mapper_gmapping | libjuiz_sdk 0.2.0、推定姿勢を `map → mapper0` に書く | なし |
| G2 | mapper_gmapping | `lidar_mount` から取付位置を受け取る | G1、T2 |
| G3 | mapper_gmapping | 画面・Docker を合わせる | G2 |
| G4 | mapper_gmapping | 0.2.0 のリリース(depends `mobile_sim >=0.3.0,<0.4`) | G3、T4 |
| S1 | 両方 | (段階 2)1 本の木にする | juiz #376 と下の 3・4 |

## juiz 本体の不足点

| # | 不足 | 困ること | 提案 |
| --- | --- | --- | --- |
| 1 | ~~Transformations を含む版が未公開~~ | — | 0.2.0 で解決 |
| 2 | システムをまたぐ Transform が無い | mapper と mobile_sim は別 System | juiz #376(IPC と HTTP の両方) |
| 3 | ContainerProcess から自分以外の Container の位置を読めない | tf の lookupTransform に当たるものが無い | 読み出し専用の `lookup(target, reference)` |
| 4 | ContainerProcess から Transform の値や他の Container の位置を書けない | SLAM が `map → odom` を書けない。1 つのシミュレータが複数台を動かすのにプラグイン内の共有が要る | `set_transform(source, destination, value)`(書ける範囲を決める) |
| 5 | Pose 通知に初回の呼び出しが無い | 動かない Transform(取付位置)は一度も通知されない | 登録時に 1 回呼ぶオプション |
| 6 | C++ に 2 次元の変換が無い | cppjuiz の `juiz/pose.h` に yaw の変換が無い | `make_pose_2d`・`yaw` |

## 決めたこと

1. 段階 1 を juiz 0.2.0 で始め、段階 2 は juiz #376 の後。
2. 真の座標系の名前は `world`(`map` は SLAM の地図に空ける)。
3. 複数ロボットは番号を付ける(`robot0`・`lidar0`・`odom0`)。
4. 0.3 系では今の `robots:` と `index` つき Process を残し、0.4 で消す。
5. 不足点 3〜6 を juiz に issue として立てる。
